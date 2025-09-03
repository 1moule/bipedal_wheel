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
  double calculateSupportForce(double F, double Tp, double leg_length, double acc_z,
                               Eigen::Matrix<double, STATE_DIM, 1> x, Eigen::Matrix<double, CONTROL_DIM, 1> u,
                               const std::shared_ptr<ModelParams>& model_params);
  bool unstickDetection(const double& hip_effort, const double& knee_effort, const double& wheel_effort,
                        const double& hip_angle, const double& knee_angle, const double& leg_length,
                        const double& acc_z, const std::shared_ptr<ModelParams>& model_params,
                        Eigen::Matrix<double, STATE_DIM, 1> x);
  std::vector<hardware_interface::JointHandle*> joint_handles_;
  std::vector<control_toolbox::Pid*> pid_legs_;
  control_toolbox::Pid pid_yaw_vel_, pid_theta_diff_, pid_roll_;

  int jump_phase_ = JumpPhase::SQUAT;
  bool start_jump_ = false;
};
}  // namespace bipedal_wheel_controller
