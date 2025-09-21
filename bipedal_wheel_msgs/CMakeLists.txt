cmake_minimum_required(VERSION 3.0.2)
project(bipedal_wheel_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
)

add_message_files(
        FILES
        LocationInfo.msg
)

add_service_files(
        FILES
        Relocate.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
)