//
// Created by guanlin on 25-9-28.
//

#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>

#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "trajectory_tracker/interface/definitions.h"
#include "trajectory_tracker/interface/referenceManager/PathProcessor.h"

namespace trajectory_tracker {
using ocs2::scalar_t;
using ocs2::vector_t;

struct Pose {
  Eigen::Vector2d position;
  double yaw{};
};

class RosReferenceManager : public ocs2::ReferenceManagerDecorator {
 public:
  RosReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr);
  ~RosReferenceManager() override = default;

  void subscribe(ros::NodeHandle &nodeHandle);
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &initState) override;

 private:
   double normalizeAngle(double angle)
   {
     angle = std::fmod(angle + M_PI, 2.0 * M_PI);
     return (angle <= 0.0) ? angle + M_PI : angle - M_PI;
   }

  ::ros::Subscriber trajSub_;
  std::mutex trajMutex_;
  std::atomic_bool trajUpdated_;
  nav_msgs::Path traj_;

  ::ros::Subscriber odomSub_;

  std::mutex odomMutex_;
  std::atomic_bool odomUpdated_;
  nav_msgs::Odometry odom_;

  std::unique_ptr<trajectory_tracker::PathProcessor> pathProcessor_;
};
}  // namespace trajectory_tracker
