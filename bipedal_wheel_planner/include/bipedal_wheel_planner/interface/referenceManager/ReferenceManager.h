//
// Created by guanlin on 25-9-28.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>

#include "bipedal_wheel_planner/interface/definitions.h"
#include "bipedal_wheel_planner/interface/referenceManager/PathProcessor.h"

namespace bipedal_wheel_planner
{
using ocs2::scalar_t;
using ocs2::vector_t;

class RosReferenceManager : public ocs2::ReferenceManagerDecorator
{
public:
  RosReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr);
  ~RosReferenceManager() override = default;

  void subscribe(ros::NodeHandle & nodeHandle);
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t & initState) override;

private:
  ::ros::Subscriber pathSub_;
  std::mutex pathMutex_;
  nav_msgs::Path globalPath_;

  ::ros::Subscriber odomSub_;
  std::mutex odomMutex_;
  std::atomic_bool odomUpdated_;
  nav_msgs::Odometry odom_;

  ros::Publisher optimizedPathPub_;

  std::unique_ptr<bipedal_wheel_planner::PathProcessor> pathProcessor_;
};
}  // namespace bipedal_wheel_planner
