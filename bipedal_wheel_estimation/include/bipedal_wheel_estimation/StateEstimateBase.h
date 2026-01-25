//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <Eigen/Dense>

namespace bipedal_wheel_estimation
{
class StateEstimateBase
{
public:
  StateEstimateBase();

  virtual Eigen::Matrix<double, Eigen::Dynamic, 1> update(const ros::Time& time, const ros::Duration& period) = 0;

protected:
  void updateAngular(const Eigen::Matrix<double, 3, 1>& zyx, const Eigen::Matrix<double, Eigen::Dynamic, 1>& angularVel);
  void updateLinear(const Eigen::Matrix<double, Eigen::Dynamic, 1>& pos,
                    const Eigen::Matrix<double, Eigen::Dynamic, 1>& linearVel);
  void publishMsgs(const nav_msgs::Odometry& odom);

  Eigen::Matrix<double, Eigen::Dynamic, 1> rbdState_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tfPub_;
  ros::Time lastPub_;
};

template <typename T>
T square(T a)
{
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q)
{
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

}  // namespace bipedal_wheel_estimation
