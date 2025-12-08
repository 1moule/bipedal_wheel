//
// Created by guanlin on 25-9-28.
//

#include "trajectory_tracker/tracker.h"

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_sqp/SqpMpc.h>
#include <tf/tf.h>

namespace trajectory_tracker {
Tracker::Tracker(ros::NodeHandle &nh) {
  // Initialize OCS2
  std::string taskFile;
  std::string libFolder;
  nh.getParam("/taskFile", taskFile);
  nh.getParam("/libFolder", libFolder);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10));
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  ackerman_interface_ = std::make_shared<AckermanInterface>(taskFile, libFolder);
  setupMpc(nh);
  setupMrt();
  initMpc();

  cmdVelPublisher=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void Tracker::initMpc() {
  // Initial state
  currentObservation_.state.setZero(STATE_DIM);
  currentObservation_.input.setZero(INPUT_DIM);

  TargetTrajectories target_trajectories(
      {currentObservation_.time}, {ackerman_interface_->getInitialState()}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(ackerman_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void Tracker::update() {
  // Update the current state of the system
  try
  {
    auto pose = tf_buffer_->lookupTransform("map", "base_link", ros::Time(0));
    currentObservation_.state(0) = pose.transform.translation.x;
    currentObservation_.state(1) = pose.transform.translation.y;
    currentObservation_.state(2) = tf::getYaw(pose.transform.rotation);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  currentObservation_.time += 0.01;
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(
      currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput,
      plannedMode);

  currentObservation_.input = optimizedInput;
  observationPublisher_.publish(
      ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));

  // Publish cmd vel
  geometry_msgs::Twist twist;
  twist.linear.x = optimizedInput(0);
  twist.angular.z = optimizedInput(1);
  cmdVelPublisher.publish(twist);
}

void Tracker::setupMpc(ros::NodeHandle &nh) {
  mpc_ = std::make_shared<SqpMpc>(
      ackerman_interface_->mpcSettings(), ackerman_interface_->sqpSettings(),
      ackerman_interface_->getOptimalControlProblem(), ackerman_interface_->getInitializer());
  auto rosReferenceManagerPtr = std::make_shared<trajectory_tracker::RosReferenceManager>(
      ackerman_interface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  std::string robotName = "ackerman";
  observationPublisher_ =
      nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void Tracker::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&ackerman_interface_->getRollout());
  mpcTimer_.reset();

  trackerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (trackerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            ackerman_interface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception &e) {
        trackerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      }
    }
  });
  setThreadPriority(ackerman_interface_->sqpSettings().threadPriority, mpcThread_);
}

}  // namespace trajectory_tracker
