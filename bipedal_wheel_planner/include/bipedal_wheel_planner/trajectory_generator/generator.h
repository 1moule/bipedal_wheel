//
// Created by guanlin on 25-12-10.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "bipedal_wheel_planner/trajectory_generator/backend_tools/TrajectoryOptimizer.hpp"
#include "bipedal_wheel_planner/trajectory_generator/frontend_tools/Astar.hpp"
#include "bipedal_wheel_planner/trajectory_generator/perception_tools/GridMap.hpp"

namespace bipedal_wheel_planner
{
class TrajectoryGenerator
{
public:
  TrajectoryGenerator(ros::NodeHandle & nh);
  ~TrajectoryGenerator() = default;

private:
  std::shared_ptr<grid_map::GridMap> gridMap_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  std::unique_ptr<path_planning::AStar> astar_;

  ros::Subscriber gridMapSub_, goalSub_;
  ros::Publisher aStarPathPub_;

  Eigen::Vector2d currentPos{0, 0}, goal_;
};
}  // namespace bipedal_wheel_planner
