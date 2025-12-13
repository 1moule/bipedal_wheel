cmake_minimum_required(VERSION 3.0.2)
project(bipedal_wheel_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        geometry_msgs
)

add_message_files(
        FILES
        ActuatorState.msg
        LocationInfo.msg
        Trajectory.msg
)

add_service_files(
        FILES
        Relocate.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        geometry_msgs
)