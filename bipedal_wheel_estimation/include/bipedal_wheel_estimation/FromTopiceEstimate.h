//
// Created by qiayuan on 2022/7/24.
//

#include <realtime_tools/realtime_buffer.h>

#include "bipedal_wheel_estimation/StateEstimateBase.h"

#pragma once
namespace bipedal_wheel_estimation
{

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate();

  Eigen::Matrix<double, Eigen::Dynamic, 1> update(const ros::Time& time, const ros::Duration& period) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace bipedal_wheel_estimation
