//
// Created by guanlin on 25-9-15.
//

#include <ros/ros.h>

#include "bipedal_wheel_hw/bipedal_wheel_hw_loop.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bipedal_wheel_hw");
  ros::NodeHandle nh;

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(2);
  spinner.start();

  try {
    // Create the hardware interface specific to your robot
    std::shared_ptr<bipedal_wheel_hw::BipedalWheelHW> hw_interface = std::make_shared<bipedal_wheel_hw::BipedalWheelHW>();

    // Start the control loop
    bipedal_wheel_hw::BipedalWheelHWLoop control_loop(nh, hw_interface);

    // Wait until shutdown signal received
    ros::waitForShutdown();
  } catch (const ros::Exception & e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n" << "\t" << e.what());
    return 1;
  }

  return 0;
}
