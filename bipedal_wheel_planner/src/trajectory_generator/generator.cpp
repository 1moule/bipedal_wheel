//
// Created by guanlin on 25-12-10.
//

#include "bipedal_wheel_planner/trajectory_generator/generator.h"

#include <bipedal_wheel_msgs/Trajectory.h>
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
    Eigen::Vector2d origin(msg->info.origin.position.x, msg->info.origin.position.y);
    gridMap_->init(size_x, size_y, resolution, origin);
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
  astar_ = std::make_unique<path_planning::AStar>(*gridMap_, 0.3);
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
    auto aStarPath = astar_->planWithPostProcessing(currentPos, goal_, 5000);

    // Trajectory Optimization
    TrajOpt::TrajectoryParams params;
    params.piece_len = aStarPath.total_length / aStarPath.total_time;
    params.total_time = aStarPath.total_time;
    params.total_len = aStarPath.total_length;
    optimizer_ =
      std::make_unique<TrajOpt::TrajectoryOptimizer>(gridMap_, aStarPath.optimized_path, params);
    if (!optimizer_->plan()) std::cerr << "Trajectory optimization failed!" << std::endl;

    // Publish entire trajectory
    nav_msgs::Path path;
    path.header = msg->header;
    auto trajectory = optimizer_->sampleTrajectory(0.1);  // Sample every 0.1s
    for (const auto & t : trajectory) {
      geometry_msgs::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position.x = t.x();
      pose.pose.position.y = t.y();
      pose.pose.position.z = 0.;
      pose.pose.orientation.w = 1;
      path.poses.push_back(pose);
    }
    aStarPathPub_.publish(path);
    trajectoryTime_ = 0.;
    targetUpdate_ = true;
  };
  goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);

  referenceTrajectoryPub_ =
    nh.advertise<bipedal_wheel_msgs::Trajectory>("/reference_trajectory", 1);
  referencePosPub_ = nh.advertise<nav_msgs::Path>("/reference_pos", 1);
}

void TrajectoryGenerator::update(const ros::Duration & duraction)
{
  if (optimizer_ != nullptr) {
    // Get current position
    Eigen::Vector2d currentPos;
    try {
      auto pose = tfBuffer_->lookupTransform("map", "base_link", ros::Time(0));
      currentPos << pose.transform.translation.x, pose.transform.translation.y;
    } catch (tf2::TransformException & ex) {
      ROS_WARN("Could not get robot pose: %s", ex.what());
      return;
    }

    // Find nearest point in trajectory
    double closestPointTime = trajectoryTime_;
    double minSquaredDistance = std::numeric_limits<double>::max();
    const auto & trajectory = optimizer_->getOptimizedTrajectory();
    double totalTime = trajectory.getEndTime();
    for (double t = 0; t < totalTime; t += 0.1) {
      Eigen::Vector2d pointOnTrajectory = trajectory.evaluate(t);
      double currentSquaredDistance = (pointOnTrajectory - currentPos).squaredNorm();
      if (currentSquaredDistance < minSquaredDistance) {
        minSquaredDistance = currentSquaredDistance;
        closestPointTime = t;
      }
    }
    trajectoryTime_ = closestPointTime;

    // Set star time and end time
    double startTime = std::min(trajectoryTime_, totalTime);
    double endTime = std::min(startTime + 1.0, totalTime);

    // Get reference trajectory
    auto posReference = trajectory.evaluate(startTime, endTime, 0.1);     // Sample every 0.1s
    auto velReference = trajectory.evaluate(startTime, endTime, 0.1, 1);  // Sample every 0.1s
    auto accReference = trajectory.evaluate(startTime, endTime, 0.1, 2);  // Sample every 0.1s

    // Publish reference trajectory
    bipedal_wheel_msgs::Trajectory msg;
    for (int i = 0; i < posReference.size(); ++i) {
      // pos
      geometry_msgs::Pose pose;
      pose.position.x = posReference[i].x();
      pose.position.y = posReference[i].y();
      pose.position.z = 0.;
      double yaw = atan2(velReference[i].y(), velReference[i].x());
      pose.orientation.z = std::sin(yaw * 0.5);
      pose.orientation.w = std::cos(yaw * 0.5);
      msg.pos.push_back(pose);

      // vel
      geometry_msgs::Twist vel;
      vel.linear.x = velReference[i].x();
      vel.linear.y = velReference[i].y();
      vel.angular.z =
        (velReference[i].x() * accReference[i].y() - velReference[i].y() * accReference[i].x()) /
        (velReference[i].x() * velReference[i].x() + velReference[i].y() * velReference[i].y());
      msg.vel.push_back(vel);

      // acc
      geometry_msgs::Twist acc;
      acc.linear.x = accReference[i].x();
      acc.linear.y = accReference[i].y();
      msg.acc.push_back(acc);

      // time
      msg.time.push_back(startTime + i * 0.1);
    }
    msg.targetUpdate = targetUpdate_;
    referenceTrajectoryPub_.publish(msg);
    targetUpdate_ = false;

    // Publish reference position trajectory
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    for (const auto & t : msg.pos) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = t.position.x;
      pose.pose.position.y = t.position.y;
      pose.pose.position.z = 0.;
      pose.pose.orientation.w = 1;
      path.poses.push_back(pose);
    }
    referencePosPub_.publish(path);
  }
}

}  // namespace bipedal_wheel_planner
