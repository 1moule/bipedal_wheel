//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller.h"

namespace bipedal_wheel_controller
{
Recover::Recover(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
}

void Recover::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter RECOVER");
    controller->setStateChange(true);
  }

  int left_leg_state, right_leg_state;
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  detectLegState(x_left_, left_leg_state);
  detectLegState(x_right_, right_leg_state);
  if (controller->getOverturn() && left_leg_state != LegState::FRONT)
    left_cmd = computePidLegCommand(0.4, -M_PI / 2 + 0.2, left_pos_[0], left_pos_[1], *pid_legs_[0], *pid_thetas_[0],
                                    left_angle_, period);
  if (controller->getOverturn() && right_leg_state != LegState::FRONT)
    right_cmd = computePidLegCommand(0.4, -M_PI / 2 + 0.2, right_pos_[0], right_pos_[1], *pid_legs_[1], *pid_thetas_[1],
                                     right_angle_, period);
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if ((controller->getOverturn() && left_leg_state == LegState::FRONT && right_leg_state == LegState::FRONT) ||
      !controller->getOverturn())
  {
    controller->setMode(BalanceMode::STAND_UP);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit RECOVER");
  }
}
}  // namespace bipedal_wheel_controller
