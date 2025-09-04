//
// Created by guanlin on 25-9-4.
//

#include "bipedal_wheel_controller/controller_mode/mode_manager.h"

namespace bipedal_wheel_controller
{
ModeManager::ModeManager(ros::NodeHandle& controller_nh,
                         const std::vector<hardware_interface::JointHandle*>& joint_handles)
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
      ROS_ERROR("Failed to load pid %s", e.first);
  pid_wheels_.push_back(&pid_left_wheel_vel_);
  pid_wheels_.push_back(&pid_right_wheel_vel_);
  pid_legs_.push_back(&pid_left_leg_);
  pid_legs_.push_back(&pid_right_leg_);
  pid_thetas_.push_back(&pid_left_leg_theta_);
  pid_thetas_.push_back(&pid_right_leg_theta_);

  mode_map_.insert(std::make_pair(BalanceMode::NORMAL, std::make_unique<Normal>(joint_handles, pid_legs_, pid_yaw_vel_,
                                                                                pid_theta_diff_, pid_roll_)));
  mode_map_.insert(
      std::make_pair(BalanceMode::STAND_UP, std::make_unique<StandUp>(joint_handles, pid_legs_, pid_thetas_)));
  mode_map_.insert(
      std::make_pair(BalanceMode::RECOVER, std::make_unique<Recover>(joint_handles, pid_legs_, pid_thetas_)));
  mode_map_.insert(std::make_pair(BalanceMode::SIT_DOWN, std::make_unique<SitDown>(joint_handles, pid_wheels_)));
}
}  // namespace bipedal_wheel_controller
