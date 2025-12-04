cmake_minimum_required(VERSION 3.0.2)
project(bipedal_wheel_hw)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimize
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)
## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp

        bipedal_wheel_common
        bipedal_wheel_msgs

        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        realtime_tools
        urdf
)

## Find system libraries
#find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}_loader_plugins
        CATKIN_DEPENDS
        roscpp

        bipedal_wheel_common
        bipedal_wheel_msgs

        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        realtime_tools
        urdf
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/can_interface/socketcan.cpp
        src/can_interface/can_bus.cpp
        src/can_interface/can_frame_codec.cpp
        src/can_interface/configure.cpp
        src/bipedal_wheel_hw_loop.cpp
        src/bipedal_wheel_hw.cpp
        src/gpio_interface/gpio_manager.cpp
        src/parse.cpp
        src/main.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)