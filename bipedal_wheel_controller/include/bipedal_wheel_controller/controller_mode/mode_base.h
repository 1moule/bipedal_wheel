//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <ros/time.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>

#include "bipedal_wheel_controller/definitions.h"

namespace bipedal_wheel_controller
{

class BipedalController;

class ModeBase
{
public:
  virtual void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) = 0;
  virtual const char* name() const = 0;
  virtual ~ModeBase() = default;
  void updateEstimation(Eigen::Matrix<double, STATE_DIM, 1> x_left, Eigen::Matrix<double, STATE_DIM, 1> x_right)
  {
    x_left_ = x_left;
    x_right_ = x_right;
  }
  void updateLegKinematics(double* left_angle, double* right_angle, double* left_pos, double* left_spd,
                           double* right_pos, double* right_spd)
  {
    std::memcpy(left_pos_, left_pos, 2 * sizeof(double));
    std::memcpy(left_spd_, left_spd, 2 * sizeof(double));
    std::memcpy(right_pos_, right_pos, 2 * sizeof(double));
    std::memcpy(right_spd_, right_spd, 2 * sizeof(double));
    std::memcpy(left_angle_, left_angle, 2 * sizeof(double));
    std::memcpy(right_angle_, right_angle, 2 * sizeof(double));
  }
  void updateBaseState(geometry_msgs::Vector3 angular_vel_base, geometry_msgs::Vector3 linear_acc_base, double roll,
                       double pitch, double yaw)
  {
    angular_vel_base_ = angular_vel_base;
    linear_acc_base_ = linear_acc_base;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
  }

protected:
  Eigen::Matrix<double, STATE_DIM, 1> x_left_, x_right_;
  double left_angle_[2], right_angle_[2], left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];
  geometry_msgs::Vector3 angular_vel_base_{}, linear_acc_base_{};
  double roll_, pitch_, yaw_;
};

}  // namespace bipedal_wheel_controller
