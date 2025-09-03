//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/sit_down.h"
#include "bipedal_wheel_controller/controller.h"

namespace bipedal_wheel_controller
{
void SitDown::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter SIT_DOWN");
    controller->setStateChange(true);
  }

  auto joint_handles = controller->getJointHandles();
  auto pid_wheels = controller->getPidWheels();
  auto x = controller->getLeftState();
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  double left_wheel_cmd = pid_wheels[0]->computeCommand(joint_handles[0]->getVelocity(), period);
  double right_wheel_cmd = pid_wheels[1]->computeCommand(joint_handles[1]->getVelocity(), period);
  setJointCommands(joint_handles, left_cmd, right_cmd, left_wheel_cmd, right_wheel_cmd);

  if (abs(x(1)) < 0.1 && abs(x(5)) < 0.1 && abs(x(3)) < 0.15)
  {
    if (!controller->getOverturn())
      controller->setMode(BalanceMode::STAND_UP);
    else
      controller->setMode(BalanceMode::RECOVER);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit SIT_DOWN");
  }
}
}  // namespace bipedal_wheel_controller
