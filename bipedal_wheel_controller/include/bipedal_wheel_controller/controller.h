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
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/controller_mode/sit_down.h"
#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller_mode/normal.h"

namespace bipedal_wheel_controller
{
using Eigen::Matrix;

class BipedalController
  : public controller_interface::MultiInterfaceController<hardware_interface::ImuSensorInterface,
                                                          hardware_interface::EffortJointInterface>
{
public:
  BipedalController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // clang-format off
  bool getOverturn(){ return overturn_; }
  bool getStateChange(){ return balance_state_changed_; }
  bool getCompleteStand(){ return complete_stand_; }
  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> getCoeffs() { return coeffs_; }
  const std::shared_ptr<ModelParams>& getModelParams(){ return model_params_; }
  double getLegCmd(){ return legCmd_.data; }
  double getJumpCmd(){ return jumpCmd_.data; }
  geometry_msgs::Vector3 getVelCmd(){ return ramp_vel_cmd_; }

  void setStateChange(bool state){ balance_state_changed_ = state; }
  void setCompleteStand(bool state){ complete_stand_ = state; }
  void setJumpCmd(bool cmd){ jumpCmd_.data = cmd; }
  void setMode(int mode){ balance_mode_ = mode; }
  // clang-format on

private:
  void updateEstimation(const ros::Time& time, const ros::Duration& period);
  void updateControllerMode();
  bool setupModelParams(ros::NodeHandle& controller_nh);
  bool setupPID(ros::NodeHandle& controller_nh);
  bool setupLQR(ros::NodeHandle& controller_nh);
  void polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
               Eigen::Matrix<double, 4, 12>& coeffs);
  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> coeffs_;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> q_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};

  std::shared_ptr<ModelParams> model_params_;

  int balance_mode_ = BalanceMode::SIT_DOWN;
  bool balance_state_changed_ = false;
  std::unique_ptr<ModeBase> mode_impl;

  // stand up
  bool complete_stand_ = false, overturn_ = false;

  // handles
  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_first_leg_joint_handle_,
      left_second_leg_joint_handle_, right_first_leg_joint_handle_, right_second_leg_joint_handle_;
  std::vector<hardware_interface::JointHandle*> joint_handles_;

  // pid
  control_toolbox::Pid pid_yaw_vel_, pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_roll_;
  control_toolbox::Pid pid_left_leg_theta_, pid_right_leg_theta_;
  control_toolbox::Pid pid_left_wheel_vel_, pid_right_wheel_vel_;
  std::vector<control_toolbox::Pid*> pid_wheels_, pid_legs_, pid_thetas_;

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
};
}  // namespace bipedal_wheel_controller
