//
// Created by guanlin on 25-12-8.
//

#pragma once

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace trajectory_tracker
{
struct LookAheadResult
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double curvature = 0.0;
  bool isValid = false;
};

class PathProcessor
{
public:
  static nav_msgs::Path prunePath(
    const nav_msgs::Path & globalPath, const geometry_msgs::Point & robotPosition)
  {
    if (globalPath.poses.empty()) return globalPath;

    double minDistanceSquared = std::numeric_limits<double>::max();
    size_t closestPointIndex = 0;

    for (size_t i = 0; i < globalPath.poses.size(); ++i) {
      double diffX = globalPath.poses[i].pose.position.x - robotPosition.x;
      double diffY = globalPath.poses[i].pose.position.y - robotPosition.y;
      double currentDistanceSquared = diffX * diffX + diffY * diffY;

      if (currentDistanceSquared < minDistanceSquared) {
        minDistanceSquared = currentDistanceSquared;
        closestPointIndex = i;
      }
    }

    nav_msgs::Path prunedPath;
    prunedPath.header = globalPath.header;

    size_t remainingPointsCount = globalPath.poses.size() - closestPointIndex;
    if (remainingPointsCount > 0) {
      prunedPath.poses.reserve(remainingPointsCount);
      prunedPath.poses.insert(
        prunedPath.poses.end(), globalPath.poses.begin() + closestPointIndex,
        globalPath.poses.end());
    }

    return prunedPath;
  }

  static LookAheadResult computeLookAheadPoint(
    const nav_msgs::Path & path, const geometry_msgs::Point & robotPosition,
    double lookaheadDistance)
  {
    LookAheadResult result;
    if (path.poses.empty()) return result;

    double lookaheadDistanceSquared = lookaheadDistance * lookaheadDistance;

    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & currentPoint = path.poses[i].pose.position;
      double diffX = currentPoint.x - robotPosition.x;
      double diffY = currentPoint.y - robotPosition.y;
      double distanceToRobotSquared = diffX * diffX + diffY * diffY;

      if (distanceToRobotSquared >= lookaheadDistanceSquared) {
        if (i == 0) {
          result.x = currentPoint.x;
          result.y = currentPoint.y;
          result.theta = tf2::getYaw(path.poses[i].pose.orientation);
          result.curvature = 0.0;
          result.isValid = true;
          return result;
        }

        const auto & segmentStartPoint = path.poses[i - 1].pose.position;
        const auto & segmentEndPoint = path.poses[i].pose.position;

        double segmentVectorX = segmentEndPoint.x - segmentStartPoint.x;
        double segmentVectorY = segmentEndPoint.y - segmentStartPoint.y;
        double robotToStartVectorX = segmentStartPoint.x - robotPosition.x;
        double robotToStartVectorY = segmentStartPoint.y - robotPosition.y;

        double coeffA = segmentVectorX * segmentVectorX + segmentVectorY * segmentVectorY;
        double coeffB =
          2.0 * (robotToStartVectorX * segmentVectorX + robotToStartVectorY * segmentVectorY);
        double coeffC =
          (robotToStartVectorX * robotToStartVectorX + robotToStartVectorY * robotToStartVectorY) -
          lookaheadDistanceSquared;

        double discriminant = coeffB * coeffB - 4 * coeffA * coeffC;
        double interpolationFactor = 0.0;

        if (discriminant >= 0 && coeffA != 0) {
          interpolationFactor = (-coeffB + std::sqrt(discriminant)) / (2.0 * coeffA);
          interpolationFactor = std::max(0.0, std::min(1.0, interpolationFactor));
        }

        result.x = segmentStartPoint.x + interpolationFactor * segmentVectorX;
        result.y = segmentStartPoint.y + interpolationFactor * segmentVectorY;

        result.theta = std::atan2(segmentVectorY, segmentVectorX);

        if (i + 1 < path.poses.size()) {
          const auto & nextPoint = path.poses[i + 1].pose.position;
          result.curvature =
            calculateMengerCurvature(segmentStartPoint, segmentEndPoint, nextPoint);
        } else {
          result.curvature = 0.0;
        }

        result.isValid = true;
        return result;
      }
    }

    const auto & finalPose = path.poses.back();
    result.x = finalPose.pose.position.x;
    result.y = finalPose.pose.position.y;
    result.theta = tf2::getYaw(finalPose.pose.orientation);
    result.curvature = 0.0;
    result.isValid = true;

    return result;
  }

private:
  static double calculateMengerCurvature(
    const geometry_msgs::Point & pointA, const geometry_msgs::Point & pointB,
    const geometry_msgs::Point & pointC)
  {
    double signedArea = 0.5 * (pointA.x * (pointB.y - pointC.y) + pointB.x * (pointC.y - pointA.y) +
                               pointC.x * (pointA.y - pointB.y));

    double lengthAB = std::hypot(pointA.x - pointB.x, pointA.y - pointB.y);
    double lengthBC = std::hypot(pointB.x - pointC.x, pointB.y - pointC.y);
    double lengthAC = std::hypot(pointA.x - pointC.x, pointA.y - pointC.y);

    if (lengthAB < 1e-4 || lengthBC < 1e-4 || lengthAC < 1e-4) return 0.0;
    double curvature = (4.0 * signedArea) / (lengthAB * lengthBC * lengthAC);

    return curvature;
  }
};
}  // namespace trajectory_tracker
