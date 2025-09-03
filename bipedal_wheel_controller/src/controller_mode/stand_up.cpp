//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"

namespace bipedal_wheel_controller
{
StandUp::StandUp(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
}

void StandUp::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  auto x_left_ = controller->getLeftState();
  auto x_right_ = controller->getRightState();
  auto left_pos_ = controller->getLeftPos();
  auto right_pos_ = controller->getRightPos();
  auto left_angle = controller->getLeftAngle();
  auto right_angle = controller->getRightAngle();

  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter STAND_UP");
    controller->setStateChange(true);
    controller->setCompleteStand(false);
    detectLegState(x_left_, left_leg_state);
    detectLegState(x_right_, right_leg_state);
  }

  double theta_des_l, theta_des_r, length_des_l, length_des_r;
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  setUpLegMotion(x_left_, right_leg_state, left_pos_[0], left_pos_[1], left_leg_state, theta_des_l, length_des_l);
  setUpLegMotion(x_right_, left_leg_state, right_pos_[0], right_pos_[1], right_leg_state, theta_des_r, length_des_r);
  left_cmd = computePidLegCommand(length_des_l, theta_des_l, left_pos_[0], left_pos_[1], *pid_legs_[0], *pid_thetas_[0],
                                  left_angle, period);
  right_cmd = computePidLegCommand(length_des_r, theta_des_r, right_pos_[0], right_pos_[1], *pid_legs_[1],
                                   *pid_thetas_[1], right_angle, period);
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  if (((left_pos_[1] < 0. && left_leg_state == LegState::BEHIND) ||
       (left_pos_[1] > 0. && left_leg_state == LegState::UNDER)) &&
      ((right_pos_[1] < 0. && right_leg_state == LegState::BEHIND) ||
       (right_pos_[1] > 0. && right_leg_state == LegState::UNDER)))
  {
    controller->setMode(BalanceMode::NORMAL);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit STAND_UP");
  }
}
}  // namespace bipedal_wheel_controller
