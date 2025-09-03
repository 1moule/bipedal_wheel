//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"

namespace bipedal_wheel_controller
{
class Normal : public ModeBase
{
public:
  Normal(const std::vector<hardware_interface::JointHandle*>& joint_handles,
         const std::vector<control_toolbox::Pid*>& pid_legs, const control_toolbox::Pid& pid_yaw_vel,
         const control_toolbox::Pid& pid_theta_diff, const control_toolbox::Pid& pid_roll);
  void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "NORMAL";
  }

private:
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_;
  control_toolbox::Pid pid_yaw_vel_, pid_theta_diff_, pid_roll_;

  int jump_phase_ = JumpPhase::SQUAT;
  bool start_jump_ = false;
};
}  // namespace bipedal_wheel_controller
