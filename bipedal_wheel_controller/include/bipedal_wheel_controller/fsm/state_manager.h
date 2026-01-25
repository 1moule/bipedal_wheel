//
// Created by guanlin on 25-9-4.
//

#pragma once

#include "bipedal_wheel_controller/fsm/state_base.h"
#include "bipedal_wheel_controller/fsm/sit_down.h"
#include "bipedal_wheel_controller/fsm/stand_up.h"
#include "bipedal_wheel_controller/fsm/recover.h"
#include "bipedal_wheel_controller/fsm/normal.h"

namespace bipedal_wheel_controller
{
class StateManager
{
public:
  StateManager(ros::NodeHandle& controller_nh, const std::vector<hardware_interface::JointHandle*>& joint_handles);
  virtual ~StateManager() = default;
  void switchMode(int mode)
  {
    mode_impl = mode_map_[mode];
  }
  const std::shared_ptr<StateBase>& getModeImpl()
  {
    return mode_impl;
  }

private:
  std::shared_ptr<StateBase> mode_impl;
  std::map<int, std::shared_ptr<StateBase>> mode_map_;

  control_toolbox::Pid pid_yaw_vel_, pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_roll_;
  control_toolbox::Pid pid_left_leg_theta_, pid_right_leg_theta_;
  control_toolbox::Pid pid_left_wheel_vel_, pid_right_wheel_vel_;
  std::vector<control_toolbox::Pid*> pid_wheels_, pid_legs_, pid_thetas_;
};
}  // namespace bipedal_wheel_controller
