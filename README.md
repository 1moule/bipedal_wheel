# bipedal_wheel
## Introduction
This software contains simulation and basic control framework of bipedal wheel robot. In addition, navigation and trajectory tracker were added to the underlying control system.

The specific implementation method is as follows:

1. Calculating leg kinematics by VMC, Implement balance control by LQR and adding PID to achieve comprehensive motion.
2. Location is achieved using ICP and EKF, and navigation is achieved using ROS navigation.
3. Implement trajectory tracker for an NMPC problem using [OCS2](https://github.com/leggedrobotics/ocs2).

## Installation
### Source code

```
# Clone repository bipedal_wheel 
git clone https://github.com/1moule/bipedal_wheel
```

### OCS2
1. Clone OCS2 and ocs2_robotic_assets into your workspace
   
    ```
    # Clone OCS2
    git clone git@github.com:leggedrobotics/ocs2.git
    
    # Clone ocs2_robotic_assets
    git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
    ```
    
3. Compile these packages with catkin tools
   
   ```
   catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
   catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
   ```
   
### build
Build the source code by catkin tools
```
catkin build
```
## Quick Start
+ Run basic control in empty world
  1. Run the simulation:
     
     ```
     roslaunch bipedal_wheel_gazebo empty_world.launch
     ```
     
  3. Load the controller
     
     ```
     roslaunch bipedal_wheel_controller load_controller.launch
     ```
     
  5. Start controller by `rqt_controller_manager`Gui:
     
     ```
     sudo apt install ros-noetic-rqt-controller-manager
     rosrun rqt_controller_manager rqt_controller_manager
     ```
     
  7. Then you can control the robot by topic /cmd_vel 
+ Run navigation
  1. Run the simulation:
     
     ```
     roslaunch bipedal_wheel_gazebo test_world.launch
     ```
     
  2. Load and start controller as the previous steps and **DO NOT** publish topic /cmd_vel
  3. Run navigation
     
     ```
     roslaunch bipedal_wheel_navigation navigation.launch
     ```

+ Run Trajectory tracker
  1. Run the simulation:
     
     ```
     roslaunch bipedal_wheel_gazebo empty_world.launch
     ```

  2. Load and start controller as the previous steps and **DO NOT** publish topic /cmd_vel
  3. Run trajectory tracker
     
     ```
     roslaunch trajectory_tracker tracker.launch
     ```
