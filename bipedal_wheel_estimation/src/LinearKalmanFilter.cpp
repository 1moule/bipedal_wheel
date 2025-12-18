//
// Created by guanlin on 25-12-17.
//

#include "bipedal_wheel_estimation/LinearKalmanFilter.h"

namespace bipedal_wheel_estimation
{
KalmanFilterEstimate::KalmanFilterEstimate() : basePosition_(Eigen::Matrix<double, 3, 1>::Zero())
{
  xHat_.setZero(2, 1);
  p_.setIdentity(2, 2);
  a_.setIdentity(2, 2);
  a_(0, 1) = 0.001;
  b_.setZero(2, 1);
  b_ << 0.5 * 0.001 * 0.001, 0.001;
  h_.setIdentity(2, 2);
  q_.setIdentity(2, 1);
  q_ = b_ * processNoise * processNoise * b_.transpose();
  r_.setIdentity(2, 2);
  r_(0, 0) = measureNoiseVel_ * measureNoiseVel_;
  r_(1, 1) = measureNoiseAcc_ * measureNoiseAcc_;
  i_.setIdentity(2, 2);
}

void KalmanFilterEstimate::updateLegWHeelStates(const Eigen::Matrix<double, 2, 1>& legPos,
                                                const Eigen::Matrix<double, 3, 1>& legWheelVel)
{
  // length and angle
  legWheelState_.segment<2>(0) = legPos;
  //  // length dot, angle dot, wheel vel
  legWheelState_.segment<3>(2) = legWheelVel;
}

void KalmanFilterEstimate::updateImu(const Eigen::Quaternion<double>& quat,
                                     const Eigen::Matrix<double, 3, 1>& angularVelLocal,
                                     const Eigen::Matrix<double, 3, 1>& linearAccelLocal,
                                     const Eigen::Matrix<double, 3, 3>& orientationCovariance,
                                     const Eigen::Matrix<double, 3, 3>& angularVelCovariance,
                                     const Eigen::Matrix<double, 3, 3>& linearAccelCovariance)
{
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  Eigen::Matrix<double, 3, 1> zyx = quatToZyx(quat);
  updateAngular(zyx, angularVelLocal_);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> KalmanFilterEstimate::update(const ros::Time& time, const ros::Duration& period)
{
  // predict
  xHat_ = a_ * xHat_;
  p_ = a_ * p_ * a_.transpose() + q_;

  // observe
  Eigen::Matrix<double, 2, 1> z;
  double wheelAngularVelAbsolute = angularVelLocal_.y() + legWheelState_(3) + legWheelState_(4);
  double wheelVelAbsolute = wheelAngularVelAbsolute * wheelRadius_;
  double baseVelAbsolute = wheelVelAbsolute + legWheelState_(0) * legWheelState_(3) * std::cos(legWheelState_(1)) +
                           legWheelState_(2) * std::sin(legWheelState_(1));
  z << baseVelAbsolute, linearAccelLocal_.x();

  // update
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> s = h_ * p_ * h_.transpose() + r_;
  k_ = p_ * h_.transpose() * s.inverse();
  xHat_ = xHat_ + k_ * (z - h_ * xHat_);
  p_ = (i_ - k_ * h_) * p_;

  auto odom = getOdomMsg(period);
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  updateLinear(
      Eigen::Matrix<double, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
      Eigen::Matrix<double, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  return rbdState_;
}

nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg(const ros::Duration& period)
{
  nav_msgs::Odometry odom;

  //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
  Eigen::Vector3d baseVelInBase(xHat_(0), 0, 0);
  Eigen::Matrix3d rotationOdomToBase = quat_.toRotationMatrix();
  Eigen::Vector3d baseVelInOdom = rotationOdomToBase.transpose() * baseVelInBase;
  odom.twist.twist.linear.x = baseVelInOdom.x();
  odom.twist.twist.linear.y = baseVelInOdom.y();
  odom.twist.twist.linear.z = baseVelInOdom.z();
  odom.twist.twist.angular.x = angularVelLocal_.x();
  odom.twist.twist.angular.y = angularVelLocal_.y();
  odom.twist.twist.angular.z = angularVelLocal_.z();
  //  for (int i = 0; i < 3; ++i)
  //  {
  //    for (int j = 0; j < 3; ++j)
  //    {
  //      odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
  //      odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
  //    }
  //  }

  basePosition_ += baseVelInOdom * period.toSec();
  odom.pose.pose.position.x = basePosition_.x();
  odom.pose.pose.position.y = basePosition_.y();
  odom.pose.pose.position.z = basePosition_.z();
  odom.pose.pose.orientation.x = quat_.x();
  odom.pose.pose.orientation.y = quat_.y();
  odom.pose.pose.orientation.z = quat_.z();
  odom.pose.pose.orientation.w = quat_.w();
  //  for (int i = 0; i < 3; ++i)
  //  {
  //    for (int j = 0; j < 3; ++j)
  //    {
  //      odom.pose.covariance[i * 6 + j] = p_(i, j);
  //      odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
  //    }
  //  }
  return odom;
}
}  // namespace bipedal_wheel_estimation
