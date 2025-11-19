//
// Created by qiayuan on 2021/11/15.
//

#include "bipedal_wheel_estimation/StateEstimateBase.h"

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace bipedal_wheel_estimation
{

StateEstimateBase::StateEstimateBase() : rbdState_(vector_t ::Zero(2 * 6))
{
  ros::NodeHandle nh;
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(
    nh, "pose", 10));

  tfPub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(nh, "/tf", 100));
}

void StateEstimateBase::updateImu(
  const Eigen::Quaternion<scalar_t> & quat, const vector3_t & angularVelLocal,
  const vector3_t & linearAccelLocal, const matrix3_t & orientationCovariance,
  const matrix3_t & angularVelCovariance, const matrix3_t & linearAccelCovariance)
{
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
    zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(
           quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}

void StateEstimateBase::updateAngular(const vector3_t & zyx, const vector_t & angularVel)
{
  rbdState_.segment<3>(3) = zyx;
  rbdState_.segment<3>(9) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t & pos, const vector_t & linearVel)
{
  rbdState_.segment<3>(0) = pos;
  rbdState_.segment<3>(6) = linearVel;
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry & odom)
{
  ros::Time time = odom.header.stamp;
  scalar_t publishRate = 500;
  if (lastPub_ + ros::Duration(1. / publishRate) <= time) {
    lastPub_ = time;
    if (odomPub_->trylock()) {
      odomPub_->msg_ = odom;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock()) {
      posePub_->msg_.header = odom.header;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
    tf2_msgs::TFMessage message;
    geometry_msgs::TransformStamped odom2base;
    odom2base.header.stamp = odom.header.stamp;
    odom2base.header.frame_id = "odom";
    odom2base.child_frame_id = "base_link";
    odom2base.transform.translation.x = odom.pose.pose.position.x;
    odom2base.transform.translation.y = odom.pose.pose.position.y;
    odom2base.transform.translation.z = odom.pose.pose.position.z;
    odom2base.transform.rotation = odom.pose.pose.orientation;
    message.transforms.push_back(odom2base);
    if (tfPub_->trylock()) {
      tfPub_->msg_ = message;
      tfPub_->unlockAndPublish();
    }
  }
}

}  // namespace bipedal_wheel_estimation
