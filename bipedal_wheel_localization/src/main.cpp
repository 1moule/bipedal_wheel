//
// Created by guanlin on 25-9-21.
//

#include "bipedal_wheel_localization/localization.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "scan_to_map_location_node");  // 节点的名字
  bipedal_wheel_localization::Scan2MapLocation scan_to_map;

  // ros::spin();
  ros::MultiThreadedSpinner spinner(2);  // Use 2 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}
