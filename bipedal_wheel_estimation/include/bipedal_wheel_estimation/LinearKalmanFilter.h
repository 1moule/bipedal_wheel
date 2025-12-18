//
// Created by guanlin on 25-12-17.
//

#pragma once

#include "bipedal_wheel_estimation/StateEstimateBase.h"

namespace bipedal_wheel_estimation
{
class KalmanFilterEstimate : public StateEstimateBase
{
public:
  KalmanFilterEstimate();

  void updateLegWHeelStates(const Eigen::Matrix<double, 2, 1>& legPos, const Eigen::Matrix<double, 3, 1>& legWheelVel);
  void updateImu(const Eigen::Quaternion<double>& quat, const Eigen::Matrix<double, 3, 1>& angularVelLocal,
                 const Eigen::Matrix<double, 3, 1>& linearAccLocal,
                 const Eigen::Matrix<double, 3, 3>& orientationCovariance,
                 const Eigen::Matrix<double, 3, 3>& angularVelCovariance,
                 const Eigen::Matrix<double, 3, 3>& linearAccelCovariance);
  Eigen::Matrix<double, Eigen::Dynamic, 1> update(const ros::Time& time, const ros::Duration& period) override;

  Eigen::Matrix<double, Eigen::Dynamic, 1> getState() const
  {
    return xHat_;
  }

protected:
  nav_msgs::Odometry getOdomMsg(const ros::Duration& period);

  // Config
  double processNoise = 10.0;
  double measureNoiseVel_ = 1.0;
  double measureNoiseAcc_ = 0.01;

  double wheelRadius_ = 0.09;

  Eigen::Quaternion<double> quat_;
  Eigen::Matrix<double, 3, 1> angularVelLocal_, linearAccelLocal_, basePosition_;
  Eigen::Matrix<double, 3, 3> orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

private:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a_, b_, h_, i_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> q_, r_, p_, k_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> xHat_;
};
}  // namespace bipedal_wheel_estimation