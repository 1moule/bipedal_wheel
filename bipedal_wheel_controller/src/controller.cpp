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
  const std::pair<const char*, hardware_interface::JointHandle*> table[] = {
    { "left_hip_joint", &left_hip_joint_handle_ },     { "left_knee_joint", &left_knee_joint_handle_ },
    { "right_hip_joint", &right_hip_joint_handle_ },   { "right_knee_joint", &right_knee_joint_handle_ },
    { "left_wheel_joint", &left_wheel_joint_handle_ }, { "right_wheel_joint", &right_wheel_joint_handle_ }
  };
  auto* joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  for (const auto& t : table)
  {
    *t.second = joint_interface->getHandle(t.first);
    joint_handles_.push_back(t.second);
  }

  ramp_x_ = std::make_unique<RampFilter>(4., 0.001);
  ramp_w_ = std::make_unique<RampFilter>(10., 0.001);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10));
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  mode_manager_ = std::make_unique<ModeManager>(controller_nh, joint_handles_);
  model_params_ = std::make_shared<ModelParams>();
  tf_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(controller_nh, "/tf", 100));
  state_pub_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(controller_nh, "/x_left_x_right", 100));

  double q = controller_nh.param("process_noise", 1000.0);
  double r_vel = controller_nh.param("measure_noise_vel", 10.0);
  double r_acc = controller_nh.param("measure_noise_acc", 0.01);
  double acc_offset = controller_nh.param("measure_acc_offset", 0.0);

  stateEstimate_ = std::make_unique<bipedal_wheel_common::KalmanFilterEstimate>(q, r_vel, r_acc, acc_offset);

  if (!setupModelParams(controller_nh) || !setupLQR(controller_nh))
    return false;
  x_left_.setZero();
  x_right_.setZero();

  // Setup subscribers
  auto legCmdCallback = [this](const std_msgs::Float64::ConstPtr msg) { legCmd_ = *msg; };
  leg_cmd_sub_ = controller_nh.subscribe<std_msgs::Float64>("/leg_command", 1, legCmdCallback);
  auto jumpCmdCallback = [this](const std_msgs::Bool::ConstPtr msg) { jumpCmd_ = *msg; };
  jump_cmd_sub_ = controller_nh.subscribe<std_msgs::Bool>("/jump_command", 1, jumpCmdCallback);
  auto velCmdCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
    vel_cmd_ = *msg;
    cmd_update_time_ = ros::Time::now();
  };
  vel_cmd_sub_ = controller_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, velCmdCallback);

  // Setup odometry realtime publisher
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";
  odom_pub_->msg_.twist.covariance = { 0.01, 0., 0.,   0., 0.,   0., 0., 0.01, 0., 0.,   0., 0.,
                                       0.,   0., 0.01, 0., 0.,   0., 0., 0.,   0., 0.01, 0., 0.,
                                       0.,   0., 0.,   0., 0.01, 0., 0., 0.,   0., 0.,   0., 0.01 };

  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = "base_link";
  odom2base_.transform.rotation.w = 1;

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
  if (!complete_stand_)
  {
    ramp_x_->clear();
    ramp_w_->clear();
  }
  ramp_vel_cmd_.x = ramp_x_->output();
  ramp_vel_cmd_.z = ramp_w_->output();

  if (!balance_state_changed_)
    mode_manager_->switchMode(balance_mode_);
  updateEstimation(time, period);
  updateOdom(time, period);
  mode_manager_->getModeImpl()->execute(this, time, period);
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
  geometry_msgs::Vector3 linear_acc_base{};
  double roll{}, pitch{}, yaw{};
  try
  {
    tf2::doTransform(gyro, angular_vel_base_, tf_buffer_->lookupTransform("base_link", imu_handle_.getFrameId(), time));
    tf2::doTransform(acc, linear_acc_base, tf_buffer_->lookupTransform("base_link", imu_handle_.getFrameId(), time));

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
    quatToRPY(toMsg(odom2base).rotation, roll, pitch, yaw);
    odom2base_.transform.rotation = toMsg(odom2base).rotation;

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
  double left_angle[2]{}, right_angle[2]{}, left_pos[2]{}, left_spd[2]{}, right_pos[2]{}, right_spd[2]{};
  // [0]:hip_vmc_joint [1]:knee_vmc_joint
  left_angle[0] = left_hip_joint_handle_.getPosition() + M_PI / 2.;
  left_angle[1] = left_knee_joint_handle_.getPosition() - M_PI / 4.;
  right_angle[0] = right_hip_joint_handle_.getPosition() + M_PI / 2.;
  right_angle[1] = right_knee_joint_handle_.getPosition() - M_PI / 4.;
  // [0] is length, [1] is angle
  leg_pos(left_angle[0], left_angle[1], left_pos);
  leg_pos(right_angle[0], right_angle[1], right_pos);
  leg_spd(left_hip_joint_handle_.getVelocity(), left_knee_joint_handle_.getVelocity(), left_angle[0], left_angle[1],
          left_spd);
  leg_spd(right_hip_joint_handle_.getVelocity(), right_knee_joint_handle_.getVelocity(), right_angle[0], right_angle[1],
          right_spd);

  // kalman filter
  double leftWheelVel, leftVelAbsolute, rightWheelVel, rightVelAbsolute, baseVelAbsolute;
  leftWheelVel = (left_wheel_joint_handle_.getVelocity() + (angular_vel_base_.y - left_spd[1]) * cos(left_pos[1])) *
                 model_params_->r;
  rightWheelVel = (right_wheel_joint_handle_.getVelocity() + (angular_vel_base_.y - right_spd[1]) * cos(right_pos[1])) *
                  model_params_->r;
  leftVelAbsolute = leftWheelVel + left_pos[0] * left_spd[1] * cos(left_pos[1]) + left_spd[0] * sin(left_pos[1]);
  rightVelAbsolute = rightWheelVel + right_pos[0] * right_spd[1] * cos(right_pos[1]) + right_spd[0] * sin(right_pos[1]);
  baseVelAbsolute = (leftVelAbsolute + rightVelAbsolute) / 2.0;
  stateEstimate_->update(Eigen::Matrix<double, 2, 1>(baseVelAbsolute, linear_acc_base.x));

  // update state
  x_left_[4] = -pitch;
  x_left_[5] = -angular_vel_base_.y;
  x_right_ = x_left_;
  x_left_[3] =
      stateEstimate_->getState()[0] - (left_pos[0] * left_spd[1] * cos(left_pos[1]) + left_spd[0] * sin(left_pos[1]));
  x_right_[3] = stateEstimate_->getState()[0] -
                (right_pos[0] * right_spd[1] * cos(right_pos[1]) + right_spd[0] * sin(right_pos[1]));
  //  x_left_[3] =
  //      (left_wheel_joint_handle_.getVelocity() + right_wheel_joint_handle_.getVelocity()) / 2. * model_params_->r;
  //  if (abs(x_left_[3]) < 0.2 && ramp_vel_cmd_.x == 0.)
  //    x_left_[2] += x_left_[3] * period.toSec();
  //  else
  //    x_left_[2] = 0.;
  if (complete_stand_)
  {
    x_left_[2] += -(ramp_vel_cmd_.x - x_left_[3]) * period.toSec();
    x_right_[2] += -(ramp_vel_cmd_.x - x_right_[3]) * period.toSec();
  }
  else
  {
    x_left_[2] = 0.;
    x_right_[2] = 0.;
  }
  x_left_[0] = left_pos[1] + pitch;
  x_left_[1] = -left_spd[1] + angular_vel_base_.y;
  x_right_[0] = right_pos[1] + pitch;
  x_right_[1] = -right_spd[1] + angular_vel_base_.y;

  if (state_pub_ && state_pub_->trylock())
  {
    state_pub_->msg_.data.resize(12);
    Eigen::Map<Eigen::Matrix<double, 12, 1>>(state_pub_->msg_.data.data()) << x_left_, x_right_;
    state_pub_->unlockAndPublish();
  }

  mode_manager_->getModeImpl()->updateEstimation(x_left_, x_right_);
  mode_manager_->getModeImpl()->updateLegKinematics(left_angle, right_angle, left_pos, left_spd, right_pos, right_spd);
  mode_manager_->getModeImpl()->updateBaseState(angular_vel_base_, linear_acc_base, roll, pitch, yaw);
}

void BipedalController::updateOdom(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 linear_vel_base, linear_vel_odom;
  linear_vel_base.x = (x_left_[3] + x_right_[3]) / 2.;
  linear_vel_base.y = 0.;
  linear_vel_base.z = 0.;
  tf2::doTransform(linear_vel_base, linear_vel_odom, odom2base_);
  odom2base_.header.stamp = time;
  odom2base_.transform.translation.x += linear_vel_odom.x * period.toSec();
  odom2base_.transform.translation.y += linear_vel_odom.y * period.toSec();
  //  odom2base_.transform.translation.z += linear_vel_odom.z * period.toSec();
  tf2_msgs::TFMessage message;
  message.transforms.push_back(odom2base_);
  tf_buffer_->setTransform(odom2base_, "bipedal_wheel_controller", true);
  if (tf_pub_->trylock())
  {
    tf_pub_->msg_ = message;
    tf_pub_->unlockAndPublish();
  }
  if (loop_count_ % 10 == 0)
  {
    if (odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = odom2base_.transform.translation.x;
      odom_pub_->msg_.pose.pose.position.y = odom2base_.transform.translation.y;
      odom_pub_->msg_.pose.pose.position.z = odom2base_.transform.translation.z;
      odom_pub_->msg_.pose.pose.orientation = odom2base_.transform.rotation;
      odom_pub_->msg_.twist.twist.linear.x = linear_vel_base.x;
      odom_pub_->msg_.twist.twist.linear.y = 0.;
      odom_pub_->msg_.twist.twist.angular.z = angular_vel_base_.z;
      odom_pub_->unlockAndPublish();
    }
    loop_count_ = 0;
  }
  loop_count_++;
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
        { "wheel_radius", &model_params_->r } };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
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

void BipedalController::polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
                                Eigen::Matrix<double, 4, 12>& coeffs)
{
  int N = L0s.size();
  Eigen::MatrixXd A(N, 4), B(N, 12);
  for (int i = 0; i < N; ++i)
  {
    A.block(i, 0, 1, 4) << pow(L0s[i], 3), pow(L0s[i], 2), L0s[i], 1.0;
    Eigen::Map<const Eigen::Matrix<double, 12, 1>> flat(Ks[i].data());
    B.row(i) = flat.transpose();
  }
  coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * B);
}

}  // namespace bipedal_wheel_controller
PLUGINLIB_EXPORT_CLASS(bipedal_wheel_controller::BipedalController, controller_interface::ControllerBase)
