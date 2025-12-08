//
// Created by guanlin on 25-9-28.
//

#include "trajectory_tracker/interface/referenceManager/ReferenceManager.h"

#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <tf/tf.h>
#include <angles/angles.h>

namespace trajectory_tracker {
RosReferenceManager::RosReferenceManager(
    std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)) {
  pathProcessor_ = std::make_unique<trajectory_tracker::PathProcessor>();
}

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(2);
  const scalar_t rotationTime = std::abs(dyaw) / 5.;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / 3;
  return std::max(rotationTime, displacementTime);
}

void RosReferenceManager::preSolverRun(
    scalar_t initTime, scalar_t finalTime, const vector_t &initState) {
  nav_msgs::Odometry odom{};
  if(odomUpdated_){
    std::lock_guard<std::mutex> lock(odomMutex_);
    odomUpdated_ = false;
    odom = odom_;
  }
  if (trajUpdated_ && !traj_.poses.empty()) {
    std::lock_guard<std::mutex> lock(trajMutex_);
    trajUpdated_ = false;

    geometry_msgs::Point currentPosition;
    currentPosition.x = initState(0);
    currentPosition.y = initState(1);
    currentPosition.z = 0.0;
    auto path = pathProcessor_->prunePath(traj_, currentPosition);
    auto lookahead_result = pathProcessor_->computeLookAheadPoint(
        path, currentPosition, 1.0);

    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;

    vector_t targetState = vector_t::Zero(STATE_DIM);
    vector_t targetInput = vector_t::Zero(INPUT_DIM);
    targetState(0) = lookahead_result.x;
    targetState(1) = lookahead_result.y;
    targetState(2) = lookahead_result.theta;
    targetInput(0) = std::hypot(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y);
    targetInput(1) = normalizeAngle(targetInput(0) * lookahead_result.kappa);

    scalar_t estimatedTimeToTarget = estimateTimeToTarget(targetState - initState);

    timeTrajectory = {initTime, initTime + estimatedTimeToTarget};
    stateTrajectory.assign(2, targetState);
    inputTrajectory.assign(2, targetInput);

    referenceManagerPtr_->setTargetTrajectories({timeTrajectory, stateTrajectory, inputTrajectory});
  }
  referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);
}

void RosReferenceManager::subscribe(ros::NodeHandle &nodeHandle) {
  auto trajCallback = [this](const nav_msgs::Path::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(trajMutex_);
    trajUpdated_ = true;
    traj_ = *msg;
  };
  trajSub_ = nodeHandle.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 1, trajCallback);

  auto odomCallback = [this](const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(odomMutex_);
    odomUpdated_ = true;
    odom_ = *msg;
  };
  odomSub_ = nodeHandle.subscribe<nav_msgs::Odometry>("/odom", 1, odomCallback);
}
}  // namespace trajectory_tracker