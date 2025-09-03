//
// Created by guanlin on 25-8-28.
//

#include "bipedal_wheel_controller/controller.h"

#include <angles/angles.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pluginlib/class_list_macros.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include "bipedal_wheel_controller/vmc/leg_conv.h"
#include "bipedal_wheel_controller/vmc/leg_spd.h"
#include "bipedal_wheel_controller/vmc/leg_pos.h"

namespace bipedal_wheel_controller
{
bool BipedalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  std::string left_wheel_joint, right_wheel_joint, left_first_leg_joint, left_second_leg_joint, right_first_leg_joint,
      right_second_leg_joint;
  const std::tuple<const char*, std::string*, hardware_interface::JointHandle*> table[] = {
    { "left/first_leg_joint", &left_first_leg_joint, &left_first_leg_joint_handle_ },
    { "left/second_leg_joint", &left_second_leg_joint, &left_second_leg_joint_handle_ },
    { "right/first_leg_joint", &right_first_leg_joint, &right_first_leg_joint_handle_ },
    { "right/second_leg_joint", &right_second_leg_joint, &right_second_leg_joint_handle_ },
    { "left/wheel_joint", &left_wheel_joint, &left_wheel_joint_handle_ },
    { "right/wheel_joint", &right_wheel_joint, &right_wheel_joint_handle_ }
  };
  auto* joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  for (const auto& t : table)
  {
    if (!controller_nh.getParam(std::get<0>(t), *std::get<1>(t)))
    {
      ROS_ERROR("Joint '%s' not found (ns: %s)", std::get<0>(t), controller_nh.getNamespace().c_str());
      return false;
    }
    *std::get<2>(t) = joint_interface->getHandle(*std::get<1>(t));
    joint_handles_.push_back(std::get<2>(t));
  }

  ramp_x_ = std::make_unique<RampFilter>(4., 0.001);
  ramp_w_ = std::make_unique<RampFilter>(10., 0.001);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10));
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  model_params_ = std::make_shared<ModelParams>();

  if (!setupModelParams(controller_nh) || !setupPID(controller_nh) || !setupLQR(controller_nh))
    return false;

  auto legCmdCallback = [this](const std_msgs::Float64::ConstPtr msg) { legCmd_ = *msg; };
  leg_cmd_sub_ = controller_nh.subscribe<std_msgs::Float64>("/leg_command", 1, legCmdCallback);
  auto jumpCmdCallback = [this](const std_msgs::Bool::ConstPtr msg) { jumpCmd_ = *msg; };
  jump_cmd_sub_ = controller_nh.subscribe<std_msgs::Bool>("/jump_command", 1, jumpCmdCallback);
  auto velCmdCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
    vel_cmd_ = *msg;
    cmd_update_time_ = ros::Time::now();
  };
  vel_cmd_sub_ = controller_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, velCmdCallback);

  return true;
}

void BipedalController::update(const ros::Time& time, const ros::Duration& period)
{
  if ((time - cmd_update_time_).toSec() > 0.1)
  {
    ramp_x_->input(0.);
    ramp_w_->input(0.);
  }
  else
  {
    ramp_x_->input(vel_cmd_.linear.x);
    ramp_w_->input(vel_cmd_.angular.z);
  }
  ramp_vel_cmd_.x = ramp_x_->output();
  ramp_vel_cmd_.z = ramp_w_->output();
  moveJoint(time, period);
}

void BipedalController::updateEstimation(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, acc;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  acc.x = imu_handle_.getLinearAcceleration()[0];
  acc.y = imu_handle_.getLinearAcceleration()[1];
  acc.z = imu_handle_.getLinearAcceleration()[2];
  tf2::Transform odom2imu, imu2base, odom2base;
  try
  {
    tf2::doTransform(gyro, angular_vel_base_, tf_buffer_->lookupTransform("base_link", imu_handle_.getFrameId(), time));
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = tf_buffer_->lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
    tf2::Quaternion odom2imu_quaternion;
    tf2::Vector3 odom2imu_origin;
    odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                                 imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
    odom2imu_origin.setValue(0, 0, 0);
    odom2imu.setOrigin(odom2imu_origin);
    odom2imu.setRotation(odom2imu_quaternion);
    odom2base = odom2imu * imu2base;
    quatToRPY(toMsg(odom2base).rotation, roll_, pitch_, yaw_);

    tf_msg.transform = tf2::toMsg(odom2imu.inverse());
    tf_msg.header.stamp = time;
    tf2::doTransform(acc, linear_acc_base_, tf_msg);

    tf2::Vector3 z_body(0, 0, 1);
    tf2::Vector3 z_world = tf2::quatRotate(odom2base.getRotation(), z_body);
    overturn_ = z_world.z() < 0;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
    return;
  }

  // vmc
  // [0]:hip_vmc_joint [1]:knee_vmc_joint
  left_angle[0] = left_first_leg_joint_handle_.getPosition() + M_PI / 2.;
  left_angle[1] = left_second_leg_joint_handle_.getPosition() - M_PI / 4.;
  right_angle[0] = right_first_leg_joint_handle_.getPosition() + M_PI / 2.;
  right_angle[1] = right_second_leg_joint_handle_.getPosition() - M_PI / 4.;
  // [0] is length, [1] is angle
  leg_pos(left_angle[0], left_angle[1], left_pos_);
  leg_pos(right_angle[0], right_angle[1], right_pos_);
  leg_spd(left_first_leg_joint_handle_.getVelocity(), left_second_leg_joint_handle_.getVelocity(), left_angle[0],
          left_angle[1], left_spd_);
  leg_spd(right_first_leg_joint_handle_.getVelocity(), right_second_leg_joint_handle_.getVelocity(), right_angle[0],
          right_angle[1], right_spd_);

  // update state
  x_left_[3] =
      (left_wheel_joint_handle_.getVelocity() + right_wheel_joint_handle_.getVelocity()) / 2.0 * model_params_->r;
  if (abs(x_left_[3]) < 0.2 && ramp_vel_cmd_.x == 0.)
    x_left_[2] += x_left_[3] * period.toSec();
  else
    x_left_[2] = 0.;
  x_left_[0] = left_pos_[1] + pitch_;
  x_left_[1] = -left_spd_[1] + angular_vel_base_.y;
  x_left_[4] = -pitch_;
  x_left_[5] = -angular_vel_base_.y;
  x_right_ = x_left_;
  x_right_[0] = right_pos_[1] + pitch_;
  x_right_[1] = -right_spd_[1] + angular_vel_base_.y;
}

void BipedalController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  updateEstimation(time, period);
  if (!balance_state_changed_)
  {
    if (balance_mode_ == BalanceMode::SIT_DOWN)
      controller_mode_ = std::make_unique<SitDown>(joint_handles_, pid_wheels_);
    else if (balance_mode_ == BalanceMode::STAND_UP)
      controller_mode_ = std::make_unique<StandUp>(joint_handles_, pid_legs_, pid_thetas_);
    else if (balance_mode_ == BalanceMode::RECOVER)
      controller_mode_ = std::make_unique<Recover>(joint_handles_, pid_legs_, pid_thetas_);
    else if (balance_mode_ == BalanceMode::NORMAL)
      controller_mode_ = std::make_unique<Normal>(joint_handles_, pid_legs_, pid_yaw_vel_, pid_theta_diff_, pid_roll_);
  }
  controller_mode_->updateEstimation(x_left_, x_right_);
  controller_mode_->updateLegKinematics(left_angle, right_angle, left_pos_, left_spd_, right_pos_, right_spd_);
  controller_mode_->updateBaseState(angular_vel_base_, linear_acc_base_, roll_, pitch_, yaw_);
  controller_mode_->execute(this, time, period);
}

void BipedalController::stopping(const ros::Time& time)
{
  balance_mode_ = BalanceMode::RECOVER;
  balance_state_changed_ = false;
  setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });

  ROS_INFO("[balance] Controller Stop");
}

bool BipedalController::setupModelParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] =  //
      { { "m_w", &model_params_->m_w },
        { "m_p", &model_params_->m_p },
        { "M", &model_params_->M },
        { "i_w", &model_params_->i_w },
        { "i_m", &model_params_->i_m },
        { "i_p", &model_params_->i_p },
        { "l", &model_params_->l },
        { "L_weight", &model_params_->L_weight },
        { "Lm_weight", &model_params_->Lm_weight },
        { "g", &model_params_->g },
        { "wheel_radius", &model_params_->r },
        { "leg_length", &leg_length_ },
        { "vmc_bias_angle", &vmc_bias_angle_ } };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

bool BipedalController::setupPID(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, control_toolbox::Pid*> pids[] = {
    { "pid_yaw_vel", &pid_yaw_vel_ },
    { "pid_left_leg", &pid_left_leg_ },
    { "pid_right_leg", &pid_right_leg_ },
    { "pid_theta_diff", &pid_theta_diff_ },
    { "pid_roll", &pid_roll_ },
    { "pid_left_leg_theta", &pid_left_leg_theta_ },
    { "pid_right_leg_theta", &pid_right_leg_theta_ },
    { "pid_left_wheel_vel", &pid_left_wheel_vel_ },
    { "pid_right_wheel_vel", &pid_right_wheel_vel_ },
  };

  for (const auto& e : pids)
    if (controller_nh.hasParam(e.first) && !e.second->init(ros::NodeHandle(controller_nh, e.first)))
      return false;

  pid_wheels_.push_back(&pid_left_wheel_vel_);
  pid_wheels_.push_back(&pid_right_wheel_vel_);
  pid_legs_.push_back(&pid_left_leg_);
  pid_legs_.push_back(&pid_right_leg_);
  pid_thetas_.push_back(&pid_left_leg_theta_);
  pid_thetas_.push_back(&pid_right_leg_theta_);

  return true;
}

bool BipedalController::setupLQR(ros::NodeHandle& controller_nh)
{
  // Set up weight matrices
  auto loadWeightMatrix = [](ros::NodeHandle& nh, const char* key, int dim) -> Eigen::VectorXd {
    std::vector<double> v;
    if (!nh.getParam(key, v) || static_cast<int>(v.size()) != dim)
      return Eigen::VectorXd::Constant(dim, std::numeric_limits<double>::quiet_NaN());
    return Eigen::VectorXd::Map(v.data(), dim);
  };
  Eigen::VectorXd q_diag = loadWeightMatrix(controller_nh, "q", STATE_DIM);
  Eigen::VectorXd r_diag = loadWeightMatrix(controller_nh, "r", CONTROL_DIM);
  if (!q_diag.allFinite() || !r_diag.allFinite())
    return false;
  q_.setZero();
  r_.setZero();
  q_.diagonal() = q_diag;
  r_.diagonal() = r_diag;

  // Continuous model \dot{x} = A x + B u
  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>> ks;
  for (int i = 5; i < 30; i++)
  {
    double length = i / 100.;
    lengths.push_back(length);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b{};
    generateAB(model_params_, a, b, length);
    Lqr<double> lqr(a, b, q_, r_);
    if (!lqr.computeK())
    {
      ROS_ERROR("Failed to compute K of LQR.");
      return false;
    }
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k = lqr.getK();
    ks.push_back(k);
  }
  polyfit(ks, lengths, coeffs_);
  return true;
}

}  // namespace bipedal_wheel_controller
PLUGINLIB_EXPORT_CLASS(bipedal_wheel_controller::BipedalController, controller_interface::ControllerBase)