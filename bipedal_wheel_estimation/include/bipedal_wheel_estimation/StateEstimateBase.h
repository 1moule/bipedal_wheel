//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

namespace bipedal_wheel_estimation
{
using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase
{
public:
  StateEstimateBase();
  virtual void updateImu(
    const Eigen::Quaternion<scalar_t> & quat, const vector3_t & angularVelLocal,
    const vector3_t & linearAccelLocal, const matrix3_t & orientationCovariance,
    const matrix3_t & angularVelCovariance, const matrix3_t & linearAccelCovariance);

  virtual vector_t update(const ros::Time & time, const ros::Duration & period) = 0;

protected:
  void updateAngular(const vector3_t & zyx, const vector_t & angularVel);
  void updateLinear(const vector_t & pos, const vector_t & linearVel);
  void publishMsgs(const nav_msgs::Odometry & odom);

  vector3_t zyxOffset_ = vector3_t::Zero();
  vector_t rbdState_;
  contact_flag_t contactFlag_{};
  Eigen::Quaternion<scalar_t> quat_;
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>>
    posePub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tfPub_;
  ros::Time lastPub_;
};

template <typename T>
T square(T a)
{
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> & q)
{
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(
    2 * (q.x() * q.y() + q.w() * q.z()),
    square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(
    2 * (q.y() * q.z() + q.w() * q.x()),
    square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

}  // namespace bipedal_wheel_estimation
