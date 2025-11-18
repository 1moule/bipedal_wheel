//
// Created by guanlin on 25-11-11.
//

#include "trajectory_tracker/tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_tracker");
  ros::NodeHandle nh("~");

  trajectory_tracker::Tracker tracker(nh);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    tracker.update();
    loop_rate.sleep();
  }
  return 0;
}