//
// Created by guanlin on 26-1-9.
//

#pragma once

#include <Eigen/Dense>

namespace bipedal_wheel_common
{
class KalmanFilterEstimate
{
public:
  KalmanFilterEstimate(double processNoise, double measureNoiseVel, double measureNoiseAcc, double measureAccOffset);

  void update(Eigen::Matrix<double, 2, 1> measurement);

  Eigen::Matrix<double, 2, 1> getState() const
  {
    return xHat_;
  }

protected:
  // Config
  double processNoise_ = 1000.0;
  double measureNoiseVel_ = 10.0;
  double measureNoiseAcc_ = 0.01;
  double measureAccOffset_ = 0.;

private:
  Eigen::Matrix<double, 2, 2> a_, h_, q_, r_, p_, k_;
  Eigen::Matrix<double, 2, 1> xHat_;
};
}  // namespace bipedal_wheel_common
