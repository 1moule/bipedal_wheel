//
// Created by qiayuan on 2022/7/24.
//

#include <realtime_tools/realtime_buffer.h>

#include "bipedal_wheel_estimation/StateEstimateBase.h"

#pragma once
namespace bipedal_wheel_estimation
{
using namespace ocs2;

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate();

  void updateImu(
    const Eigen::Quaternion<scalar_t> & quat, const vector3_t & angularVelLocal,
    const vector3_t & linearAccelLocal, const matrix3_t & orientationCovariance,
    const matrix3_t & angularVelCovariance, const matrix3_t & linearAccelCovariance) override {};

  vector_t update(const ros::Time & time, const ros::Duration & period) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr & msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace bipedal_wheel_estimation
