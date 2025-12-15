//
// Created by guanlin on 25-9-28.
//

#include "bipedal_wheel_planner/trajectory_tracker/interface/referenceManager/ReferenceManager.h"

#include <angles/angles.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <tf/tf.h>

namespace bipedal_wheel_planner
{
RosReferenceManager::RosReferenceManager(
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
: ReferenceManagerDecorator(std::move(referenceManagerPtr))
{
  pathProcessor_ = std::make_unique<bipedal_wheel_planner::PathProcessor>();
}

scalar_t estimateTimeToTarget(const vector_t & desiredBaseDisplacement)
{
  const scalar_t & dx = desiredBaseDisplacement(0);
  const scalar_t & dy = desiredBaseDisplacement(1);
  const scalar_t & dyaw = desiredBaseDisplacement(2);
  const scalar_t rotationTime = std::abs(dyaw) / 5.;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / 3;
  return std::max(rotationTime, displacementTime);
}

void RosReferenceManager::preSolverRun(
  scalar_t initTime, scalar_t finalTime, const vector_t & initState)
{
  nav_msgs::Odometry odom{};
  if (odomUpdated_) {
    std::lock_guard<std::mutex> lock(odomMutex_);
    odomUpdated_ = false;
    odom = odom_;
  }
  if (trajectoryUpdated_ && !referenceTrajectory_.pos.empty()) {
    std::lock_guard<std::mutex> lock(trajectoryMutex_);
    trajectoryUpdated_ = false;
    if (referenceTrajectory_.targetUpdate) startTime_ = initTime;

    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;

    // Set current yaw
    scalar_t currentYaw = initState(3);
    int velDirection = 1;
    if (
      abs(angles::shortest_angular_distance(
        initState(3), tf::getYaw(referenceTrajectory_.pos[0].orientation))) >
      (abs(angles::shortest_angular_distance(
        initState(3) + M_PI, tf::getYaw(referenceTrajectory_.pos[0].orientation))))) {
      currentYaw += M_PI;
      velDirection = -1;
    }

    // Insert yaw transition trajectory
    scalar_t dyaw = angles::shortest_angular_distance(
      currentYaw, tf::getYaw(referenceTrajectory_.pos[0].orientation));
    const int totalTimeStep = static_cast<int>(std::ceil(std::abs(dyaw) / 5.));
    for (int i = 0; i < totalTimeStep; i++) {
      vector_t targetState = initState;
      targetState(3) = initState(3) + (i + 1) * dyaw / totalTimeStep;
      timeTrajectory.push_back(startTime_ + (i + 1) * 0.1);
      stateTrajectory.push_back(targetState);
      inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
    }

    for (int i = 0; i < referenceTrajectory_.pos.size(); ++i) {
      // Set target state
      const vector_t targetState = [&]() {
        vector_t targetState = vector_t::Zero(STATE_DIM);
        targetState(0) = referenceTrajectory_.pos[i].position.x;
        targetState(1) = referenceTrajectory_.pos[i].position.y;
        targetState(2) =
          std::sqrt(
            referenceTrajectory_.vel[i].linear.x * referenceTrajectory_.vel[i].linear.x +
            referenceTrajectory_.vel[i].linear.y * referenceTrajectory_.vel[i].linear.y) *
          velDirection;
        targetState(3) =
          initState(3) + angles::shortest_angular_distance(
                           currentYaw, tf::getYaw(referenceTrajectory_.pos[i].orientation));
        return targetState;
      }();

      // Set target input
      const vector_t targetInput = [&]() {
        vector_t targetInput = vector_t::Zero(INPUT_DIM);
        targetInput(0) = std::sqrt(
          referenceTrajectory_.acc[i].linear.x * referenceTrajectory_.acc[i].linear.x +
          referenceTrajectory_.acc[i].linear.y * referenceTrajectory_.acc[i].linear.y);
        targetInput(1) = referenceTrajectory_.vel[i].angular.z;
        return targetInput;
      }();

      timeTrajectory.push_back(startTime_ + totalTimeStep * 0.1 + referenceTrajectory_.time[i]);
      stateTrajectory.push_back(targetState);
      inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
    }
    referenceManagerPtr_->setTargetTrajectories({timeTrajectory, stateTrajectory, inputTrajectory});
  }
  referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);
}

void RosReferenceManager::subscribe(ros::NodeHandle & nodeHandle)
{
  auto trajectoryCallback = [this](const bipedal_wheel_msgs::Trajectory::ConstPtr & msg) {
    std::lock_guard<std::mutex> lock(trajectoryMutex_);
    trajectoryUpdated_ = true;
    referenceTrajectory_ = *msg;
  };
  trajectorySub_ = nodeHandle.subscribe<bipedal_wheel_msgs::Trajectory>(
    "/reference_trajectory", 1, trajectoryCallback);

  auto odomCallback = [this](const nav_msgs::Odometry::ConstPtr & msg) {
    std::lock_guard<std::mutex> lock(odomMutex_);
    odomUpdated_ = true;
    odom_ = *msg;
  };
  odomSub_ = nodeHandle.subscribe<nav_msgs::Odometry>("/odom", 1, odomCallback);
}
}  // namespace bipedal_wheel_planner