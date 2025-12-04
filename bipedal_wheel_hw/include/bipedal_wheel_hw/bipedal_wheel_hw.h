/*******************************************************************************
* BSD 3-Clause License
*
* Copyright (c) 2021, Qiayuan Liao
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

//
// Created by qiayuan on 12/21/20.
//

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <realtime_tools/realtime_publisher.h>
#include <bipedal_wheel_msgs/ActuatorState.h>
#include <transmission_interface/transmission_interface_loader.h>

#include "bipedal_wheel_hw/can_interface/can_bus.h"
#include "bipedal_wheel_hw/gpio_interface/gpio_interface.h"
#include "bipedal_wheel_hw/gpio_interface/gpio_manager.h"

namespace bipedal_wheel_hw
{
class BipedalWheelHW : public hardware_interface::RobotHW
{
public:
  BipedalWheelHW() = default;
  ~BipedalWheelHW();
  /** \brief Get necessary params from param server. Init hardware_interface.
  *
  * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
  * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
  *
  * @param root_nh Root node-handle of a ROS node.
  * @param robot_hw_nh Node-handle for robot hardware.
  * @return True when init successful, False when failed.
  */
  bool init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh) override;
  /** \brief Comunicate with hardware. Get datas, status of robot.
  *
  * Call @ref rm_hw::CanBus::read(). Check whether temperature of actuator is too high and whether actuator is offline.
  * Propagate actuator state to joint state for the stored transmission. Set all cmd to zero to avoid crazy soft limit
  * oscillation when not controller loaded(all controllers update after read()).
  *
  * @param time Current time
  * @param period Current time - last time
  */
  void read(const ros::Time & time, const ros::Duration & period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
  *
  * Propagate joint state to actuator state for the stored
  * transmission. Limit cmd_effort into suitable value. Call @ref rm_hw::CanBus::write(). Publish actuator current state.
  *
  * @param time Current time
  * @param period Current time - last time
  */
  void write(const ros::Time & time, const ros::Duration & period) override;

  void setCanBusThreadPriority(int thread_priority);

private:
  /** \brief Check whether actuator is specified and load specified params.
  *
  * Check whether actuator is specified and load specified params.
  *
  * @param act_datas Params you want to check and load.
  * @param robot_hw_nh Root node-handle of a ROS node.
  * @return True if all params are set up.
  */
  bool parseActData(XmlRpc::XmlRpcValue & act_datas, ros::NodeHandle & robot_hw_nh);
  /** \brief Check whether some params that are related to imu are set up and load these params.
  *
  * Check whether some params that are related to imu are set up and load these params.
  *
  * @param imu_datas Params you want to check
  * @param robot_hw_nh Root node-handle of a ROS node
  * @return True if all params are set up.
  */
  bool parseImuData(XmlRpc::XmlRpcValue & imu_datas, ros::NodeHandle & robot_hw_nh) { return true; }
  /** \brief Check whether somme params that are related to gpio are set up and load these params.
  *
  * Check whether somme params that are related to gpio are set up and load these params.
  *
  * @param gpio_datas Params you want to check and load
  * @param robot_hw_nh A handle of a ROS node
  * @return True if all params are set up.
  */
  bool parseGpioData(XmlRpc::XmlRpcValue & gpio_datas, ros::NodeHandle & robot_hw_nh);
  /** \brief Load urdf of robot from param server.
  *
  * Load urdf of robot from param server.
  *
  * @param root_nh Root node-handle of a ROS node
  * @return True if successful.
  */
  bool loadUrdf(ros::NodeHandle & root_nh);

  /** \brief Set up transmission.
  *
  * Set up transmission
  *
  * @param root_nh Root node-handle of a ROS node.
  * @return True if successful.
  */
  bool setupTransmission(ros::NodeHandle & root_nh);
  /** \brief Set up joint limit.
  *
  * Set up joint limit.
  *
  * @param root_nh Root node-handle of a ROS node.
  * @return True if successful.
  */
  bool setupJointLimit(ros::NodeHandle & root_nh);
  /** \brief Publish actuator's state to a topic named "/actuator_states".
  *
  * Publish actuator's state to a topic named "/actuator_states".
  *
  * @param time Current time
  */
  void publishActuatorState(const ros::Time & time);

  ros::ServiceServer service_server_;

  bool is_actuator_specified_ = false;
  int thread_priority_;
  // Interface
  std::vector<CanBus *> can_buses_{};
  hardware_interface::ActuatorStateInterface act_state_interface_;
  hardware_interface::EffortActuatorInterface effort_act_interface_;
  hardware_interface::gpio::GpioStateInterface gpio_state_interface_;
  hardware_interface::gpio::GpioCommandInterface gpio_command_interface_;
  //  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  //  rm_control::RmImuSensorInterface rm_imu_sensor_interface_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_{};
  transmission_interface::RobotTransmissions robot_transmissions_;
  transmission_interface::ActuatorToJointStateInterface * act_to_jnt_state_{};
  transmission_interface::JointToActuatorEffortInterface * jnt_to_act_effort_{};
  joint_limits_interface::EffortJointSaturationInterface effort_jnt_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_jnt_soft_limits_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_{};

  // URDF model of the robot
  std::string urdf_string_;                  // for transmission
  std::shared_ptr<urdf::Model> urdf_model_;  // for limit

  // Actuator
  std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_{};

  // Imu
  std::unordered_map<std::string, std::unordered_map<int, ImuData>> bus_id2imu_data_{};

  // Gpio
  std::thread gpio_thread_;
  std::atomic<bool> gpio_running_{true};
  std::unique_ptr<GpioManager> gpio_manager_;
  std::vector<hardware_interface::gpio::GpioData> gpio_state_values_;
  std::vector<hardware_interface::gpio::GpioData> gpio_command_values_;

  ros::Time last_publish_time_;
  std::shared_ptr<realtime_tools::RealtimePublisher<bipedal_wheel_msgs::ActuatorState>>
    actuator_state_pub_;
};

}  // namespace bipedal_wheel_hw