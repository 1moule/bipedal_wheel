//
// Created by guanlin on 25-9-28.
//

#pragma once

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "trajectory_tracker/interface/AckermanInterface.h"
#include "trajectory_tracker/interface/referenceManager/ReferenceManager.h"

namespace trajectory_tracker {
using namespace ocs2;

class Tracker {
 public:
  Tracker(ros::NodeHandle &nh);
  ~Tracker() = default;
  void initMpc();
  void update();

 private:
  void setupMpc(ros::NodeHandle &nh);
  void setupMrt();

  // Interface
  std::shared_ptr<AckermanInterface> ackerman_interface_;

  // State Estimation
  SystemObservation currentObservation_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  std::thread mpcThread_;
  std::atomic_bool trackerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;

  // ROS
  ros::Publisher observationPublisher_, cmdVelPublisher;
  ros::Time last_observation_time_;
};
}  // namespace trajectory_tracker
