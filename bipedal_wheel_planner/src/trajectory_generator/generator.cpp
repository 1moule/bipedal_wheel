//
// Created by guanlin on 25-12-10.
//

#include "bipedal_wheel_planner/trajectory_generator/generator.h"

#include <tf/transform_datatypes.h>

namespace bipedal_wheel_planner
{
TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle & nh)
{
  // Create grid map
  gridMap_ = std::make_shared<grid_map::GridMap>();
  auto gridMapCB = [this](const nav_msgs::OccupancyGrid::ConstPtr & msg) {
    double resolution = msg->info.resolution;
    double size_x = msg->info.width * resolution;
    double size_y = msg->info.height * resolution;
    gridMap_->init(size_x, size_y, resolution);
    grid_map::RowMatrixXi map(msg->info.width, msg->info.height);
    for (int x = 0; x < msg->info.width; ++x) {
      for (int y = 0; y < msg->info.height; ++y) {
        int index = x + y * msg->info.width;
        map(x, y) = msg->data[index];
      }
    }
    gridMap_->setMap(map);
    std::cout << "Grid map received." << std::endl;
  };
  gridMapSub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, gridMapCB);

  // Create AStar planner
  astar_ = std::make_unique<path_planning::AStar>(*gridMap_, 0.1);
  aStarPathPub_ = nh.advertise<nav_msgs::Path>("/astar_path", 1);

  // Initialize tf listener to get current pose
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10));
  tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);

  // Receive goal
  auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr & msg) {
    // A* Global Search
    try {
      auto pose = tfBuffer_->lookupTransform("map", "base_link", ros::Time(0));
      currentPos << pose.transform.translation.x, pose.transform.translation.y;
    } catch (tf2::TransformException & ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    goal_ << msg->pose.position.x, msg->pose.position.y;
    auto trajectory = astar_->planWithPostProcessing(currentPos, goal_, 5000);

    // Trajectory Optimization
    TrajOpt::TrajectoryParams params;
    params.piece_len = trajectory.total_length / trajectory.total_time;
    params.total_time = trajectory.total_time;
    params.total_len = trajectory.total_length;
    TrajOpt::TrajectoryOptimizer optimizer(gridMap_, trajectory.optimized_path, params);
    if (!optimizer.plan()) std::cerr << "Trajectory optimization failed!" << std::endl;

    // Publish trajectory
    nav_msgs::Path path;
    path.header = msg->header;
    auto opt_path = optimizer.sampleTrajectory(0.1);  // Sample every 0.1s
    for (const auto & t : opt_path) {
      geometry_msgs::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position.x = t.x();
      pose.pose.position.y = t.y();
      pose.pose.position.z = 0.;
      pose.pose.orientation.w = 1;
      path.poses.push_back(pose);
    }
    aStarPathPub_.publish(path);
  };
  goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
}
}  // namespace bipedal_wheel_planner
