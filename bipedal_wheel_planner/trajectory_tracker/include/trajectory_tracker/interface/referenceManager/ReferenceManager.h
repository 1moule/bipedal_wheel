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
#include <visualization_msgs/Marker.h>
#include "trajectory_tracker/interface/definitions.h"

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
  ::ros::Subscriber goalSub_;
  std::mutex goalMutex_;
  std::atomic_bool goalUpdated_;
  geometry_msgs::PoseStamped goal_;

  ::ros::Subscriber trajSub_;
  std::mutex trajMutex_;
  std::atomic_bool trajUpdated_;
  visualization_msgs::Marker traj_;
};
}  // namespace trajectory_tracker
