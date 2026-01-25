//
// Created by guanlin on 26-1-9.
//

#include "bipedal_wheel_common/velocity_observer.h"

namespace bipedal_wheel_common
{
KalmanFilterEstimate::KalmanFilterEstimate(double processNoise, double measureNoiseVel, double measureNoiseAcc,
                                           double measureAccOffset)
  : processNoise_(processNoise)
  , measureNoiseVel_(measureNoiseVel)
  , measureNoiseAcc_(measureNoiseAcc)
  , measureAccOffset_(measureAccOffset)
{
  // clang-format off
  xHat_.setZero();
  p_.setIdentity();
  a_ << 1, 0.001,  // Discrete-time state transition matrix
        0, 1;
  h_.setIdentity();  // Observation matrix
  q_ << 0.25 * 0.001 * 0.001 * processNoise_, 0.5 * 0.001 * processNoise_,
        0.5 * 0.001 * processNoise_, processNoise_;  // Process noise covariance
  r_ << measureNoiseVel_ * measureNoiseVel_, 0,
        0, measureNoiseAcc_ * measureNoiseAcc_;  // Measurement noise covariance
  p_.setIdentity();  // Initial estimation error covariance
  // clang-format on
}

void KalmanFilterEstimate::update(Eigen::Matrix<double, 2, 1> measurement)
{
  // Predict
  xHat_ = a_ * xHat_;
  p_ = a_ * p_ * a_.transpose() + q_;

  // Update
  measurement[1] -= measureAccOffset_;
  Eigen::Matrix<double, 2, 2> s = h_ * p_ * h_.transpose() + r_;
  k_ = p_ * h_.transpose() * s.inverse();
  xHat_ = xHat_ + k_ * (measurement - h_ * xHat_);
  p_ = (Eigen::Matrix<double, 2, 2>::Identity() - k_ * h_) * p_;
}
}  // namespace bipedal_wheel_common
