//
// Created by guanlin on 25-8-28.
//

#pragma once

#include <bipedal_wheel_common/lqr.h>
#include <bipedal_wheel_common/filter.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "bipedal_wheel_controller/helper_functions.h"

namespace bipedal_wheel_controller
{
using Eigen::Matrix;

class BipedalController
  : public controller_interface::MultiInterfaceController<hardware_interface::ImuSensorInterface,
                                                          hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
    STAND_UP,
    SIT_DOWN,
  };

public:
  BipedalController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  void updateEstimation(const ros::Time& time, const ros::Duration& period);
  void moveJoint(const ros::Time& time, const ros::Duration& period);
  void normal(const ros::Time& time, const ros::Duration& period);
  void standUp(const ros::Time& time, const ros::Duration& period);
  void sitDown(const ros::Time& time, const ros::Duration& period);
  bool setupModelParams(ros::NodeHandle& controller_nh);
  bool setupPID(ros::NodeHandle& controller_nh);
  bool setupLQR(ros::NodeHandle& controller_nh);
  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> coeffs_;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> q_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_left_, x_right_;
  double vmc_bias_angle_, left_angle[2], right_angle[2], left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];

  std::unique_ptr<ModelParams> model_params_;

  int balance_mode_ = BalanceMode::STAND_UP;
  bool balance_state_changed_ = false;

  // stand up
  int left_leg_state, right_leg_state;
  bool leg_under_body_ = false, leg_front_body_ = false, leg_behind_body_ = false, complete_stand_ = false;

  // jump
  bool complete_first_shrink_ = false, complete_elongation_ = false, complete_second_shrink_ = false,
       start_jump_ = false;

  // handles
  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_first_leg_joint_handle_,
      left_second_leg_joint_handle_, right_first_leg_joint_handle_, right_second_leg_joint_handle_;

  // pid
  control_toolbox::Pid pid_yaw_vel_, pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_roll_;
  control_toolbox::Pid pid_left_leg_theta_, pid_right_leg_theta_;
  control_toolbox::Pid pid_left_wheel_vel_, pid_right_wheel_vel_;

  // transform
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS Interface
  ros::Subscriber leg_cmd_sub_, jump_cmd_sub_, vel_cmd_sub_;
  std_msgs::Float64 legCmd_{};
  std_msgs::Bool jumpCmd_{};
  geometry_msgs::Twist vel_cmd_{};
  geometry_msgs::Vector3 ramp_vel_cmd_{};
  ros::Time cmd_update_time_;

  std::unique_ptr<RampFilter> ramp_x_, ramp_w_;
  geometry_msgs::Vector3 angular_vel_base_{}, linear_acc_base_{};
  double roll_{}, pitch_{}, yaw_{};
  double leg_length_{};
};
}  // namespace bipedal_wheel_controller
