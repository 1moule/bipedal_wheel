//
// Created by guanlin on 25-11-11.
//

#include "bipedal_wheel_planner/trajectory_generator/generator.h"
#include "bipedal_wheel_planner/trajectory_tracker/tracker.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bipedal_wheel_planner");
  ros::NodeHandle nh("~");

  bipedal_wheel_planner::Tracker tracker(nh);
  bipedal_wheel_planner::TrajectoryGenerator generator(nh);

  ros::Rate loop_rate(100);
  ros::Time last_time = ros::Time::now();
  while (ros::ok()) {
    tracker.update();
    generator.update(ros::Time::now() - last_time);
    last_time = ros::Time::now();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}