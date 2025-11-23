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
}

std::vector<Pose> generateHermiteSpline(const Pose& startPose, const Pose& endPose, int numPoints) {
  if (numPoints < 2)
    return {};

  std::vector<Pose> trajectory;
  trajectory.reserve(numPoints);

  const Eigen::Vector2d& p0 = startPose.position;
  const Eigen::Vector2d& p1 = endPose.position;

  double distance = (p1 - p0).norm();

  Eigen::Vector2d m0 = distance * Eigen::Vector2d(std::cos(startPose.yaw), std::sin(startPose.yaw));
  Eigen::Vector2d m1 = distance * Eigen::Vector2d(std::cos(endPose.yaw), std::sin(endPose.yaw));

  for (int i = 0; i < numPoints; ++i) {
    double t = static_cast<double>(i) / (numPoints - 1);
    double t2 = t * t;
    double t3 = t2 * t;

    // position
    double h1 = 2 * t3 - 3 * t2 + 1;
    double h2 = -2 * t3 + 3 * t2;
    double h3 = t3 - 2 * t2 + t;
    double h4 = t3 - t2;
    Eigen::Vector2d current_position = h1 * p0 + h2 * p1 + h3 * m0 + h4 * m1;

    // velocity
    double dh1 = 6 * t2 - 6 * t;
    double dh2 = -6 * t2 + 6 * t;
    double dh3 = 3 * t2 - 4 * t + 1;
    double dh4 = 3 * t2 - 2 * t;
    Eigen::Vector2d current_velocity = dh1 * p0 + dh2 * p1 + dh3 * m0 + dh4 * m1;

    // yaw
    double current_yaw = std::atan2(current_velocity.y(), current_velocity.x());

    trajectory.push_back({current_position, current_yaw});
  }

  trajectory.back().position = endPose.position;
  trajectory.back().yaw = endPose.yaw;

  return trajectory;
}

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(2);
  const scalar_t rotationTime = std::abs(dyaw) / 2.;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / 1.5;
  return std::max(rotationTime, displacementTime);
}

void RosReferenceManager::preSolverRun(
    scalar_t initTime, scalar_t finalTime, const vector_t &initState) {
  if (goalUpdated_) {
    std::lock_guard<std::mutex> lock(goalMutex_);
    goalUpdated_ = false;
    if (trajUpdated_) {
      std::lock_guard<std::mutex> lock(trajMutex_);
      trajUpdated_ = false;
      ocs2::scalar_array_t timeTrajectory;
      ocs2::vector_array_t stateTrajectory;
      ocs2::vector_array_t inputTrajectory;

//      vector_t targetState = vector_t::Zero(STATE_DIM);
//      targetState(0) = traj_.poses.begin()->position.x;
//      targetState(1) = traj_.poses.begin()->position.y;
//      targetState(2) = tf::getYaw(traj_.poses.begin()->orientation);

//      vector_t displacement = targetState - initState;
//      displacement(2) = angles::shortest_angular_distance(initState(2), targetState(2));
//      const int totalTimeStep =
//        static_cast<int>(std::ceil(estimateTimeToTarget(displacement)) / 0.1);

//      Pose startPose, endPose;
//      startPose.position = {initState(0), initState(1)};
//      startPose.yaw = initState(2);
//      endPose.position = {targetState(0), targetState(1)};
//      endPose.yaw = targetState(2);
//      std::vector<Pose> trajectory = generateHermiteSpline(startPose, endPose, totalTimeStep);

      auto last_ref = initState;
      auto time = initTime;
      for (size_t i = 0; i < traj_.points.size(); i++) {
        const auto & point = traj_.points[i];
        Eigen::Vector3d referenceState;
        referenceState << point.x, point.y, tf::getYaw(goal_.pose.orientation);
        vector_t displacement = referenceState - last_ref;
        displacement(2) = angles::shortest_angular_distance(last_ref(2), referenceState(2));
        const int reachTargetTime = estimateTimeToTarget(displacement);
        time += reachTargetTime;
        last_ref = referenceState;

        stateTrajectory.push_back(referenceState);
        timeTrajectory.push_back(time);
        inputTrajectory.push_back(Eigen::Vector2d::Zero(2));
      }

      referenceManagerPtr_->setTargetTrajectories(
        {timeTrajectory, stateTrajectory, inputTrajectory});
    }
  }
  referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);
}

void RosReferenceManager::subscribe(ros::NodeHandle &nodeHandle) {
  auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(goalMutex_);
    goalUpdated_ = true;
    goal_ = *msg;
  };
  goalSub_ = nodeHandle.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);

  auto trajCallback = [this](const visualization_msgs::Marker::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(trajMutex_);
    trajUpdated_ = true;
    traj_ = *msg;
  };
  trajSub_ = nodeHandle.subscribe<visualization_msgs::Marker>("/ego_planner_node/optimal_list", 1, trajCallback);
}
}  // namespace trajectory_tracker