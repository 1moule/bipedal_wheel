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

#include "bipedal_wheel_hw/bipedal_wheel_hw.h"

namespace bipedal_wheel_hw
{
bool BipedalWheelHW::init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  // Parse actuator specified by user (stored on ROS parameter server)
  if (!robot_hw_nh.getParam("actuators", xml_rpc_value))
    ROS_WARN("No actuator specified");
  else if (!parseActData(xml_rpc_value, robot_hw_nh))
    return false;
  // Parse actuator specified by user (stored on ROS parameter server)
  if (!robot_hw_nh.getParam("imus", xml_rpc_value))
    ROS_WARN("No imu specified");
  else if (!parseImuData(xml_rpc_value, robot_hw_nh))
    return false;
  if (!robot_hw_nh.getParam("gpios", xml_rpc_value)) {
    ROS_WARN("No gpio specified");
    gpio_running_ = false;
  } else if (!parseGpioData(xml_rpc_value, robot_hw_nh))
    return false;
  // CAN Bus
  if (!robot_hw_nh.getParam("bus", xml_rpc_value))
    ROS_WARN("No bus specified");
  else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ASSERT(xml_rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    for (int i = 0; i < xml_rpc_value.size(); ++i) {
      std::string bus_name = xml_rpc_value[i];
      if (bus_name.find("can") != std::string::npos)
        can_buses_.push_back(new CanBus(
          bus_name,
          CanDataPtr{
            .id2act_data_ = &bus_id2act_data_[bus_name],
            .id2imu_data_ = &bus_id2imu_data_[bus_name]},
          thread_priority_));
      else
        ROS_ERROR_STREAM("Unknown bus: " << bus_name);
    }
    for (auto bus : can_buses_) bus->enable();
  }

  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }
  // Initialize transmission
  if (!setupTransmission(root_nh)) {
    ROS_ERROR("Error occurred while setting up transmission");
    return false;
  }
  // Initialize joint limit
  if (!setupJointLimit(root_nh)) {
    ROS_ERROR("Error occurred while setting up joint limit");
    return false;
  }

  // Other Interface
  registerInterface(&gpio_state_interface_);
  registerInterface(&gpio_command_interface_);

  actuator_state_pub_.reset(new realtime_tools::RealtimePublisher<bipedal_wheel_msgs::ActuatorState>(
    root_nh, "/actuator_states", 100));

  gpio_manager_ = std::make_unique<GpioManager>();
  gpio_thread_ = std::thread([&]() {
    while (gpio_running_) {
      gpio_manager_->readGpio(gpio_state_values_);
      gpio_manager_->writeGpio(gpio_command_values_);
    }
  });

  return true;
}

void BipedalWheelHW::read(const ros::Time & time, const ros::Duration & period)
{
  for (auto bus : can_buses_) bus->read(time);
  for (auto & id2act_datas : bus_id2act_data_)
    for (auto & act_data : id2act_datas.second) {
      try {  // Duration will be out of dual 32-bit range while motor failure
        act_data.second.halted =
          (time - act_data.second.stamp).toSec() > 0.1 || act_data.second.temp > 99;
      } catch (std::runtime_error & ex) {
      }
      if (act_data.second.halted) {
        act_data.second.seq = 0;
        act_data.second.vel = 0;
        act_data.second.effort = 0;
        act_data.second.calibrated = false;  // set the actuator no calibrated
      }
    }
  if (is_actuator_specified_) act_to_jnt_state_->propagate();
  // Set all cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto effort_joint_handle : effort_joint_handles_) effort_joint_handle.setCommand(0.);
}

void BipedalWheelHW::write(const ros::Time & time, const ros::Duration & period)
{
  if (is_actuator_specified_) {
    // Propagate without joint limits
    jnt_to_act_effort_->propagate();
    // Save commanded effort before enforceLimits
    for (auto & id2act_datas : bus_id2act_data_)
      for (auto & act_data : id2act_datas.second)
        act_data.second.cmd_effort = act_data.second.exe_effort;
    // enforceLimits will limit cmd_effort into suitable value https://github.com/ros-controls/ros_control/wiki/joint_limits_interface
    effort_jnt_saturation_interface_.enforceLimits(period);
    effort_jnt_soft_limits_interface_.enforceLimits(period);
    // Propagate with joint limits
    jnt_to_act_effort_->propagate();
    // Restore the cmd_effort for the calibrating joint
    for (auto & id2act_datas : bus_id2act_data_)
      for (auto & act_data : id2act_datas.second)
        if (act_data.second.need_calibration && !act_data.second.calibrated)
          act_data.second.exe_effort = act_data.second.cmd_effort;
  }
  for (auto & bus : can_buses_) bus->write();
  publishActuatorState(time);
}

void BipedalWheelHW::setCanBusThreadPriority(int thread_priority) { thread_priority_ = thread_priority; }

bool BipedalWheelHW::loadUrdf(ros::NodeHandle & root_nh)
{
  if (urdf_model_ == nullptr) urdf_model_ = std::make_shared<urdf::Model>();
  // get the urdf param on param server
  root_nh.getParam("/robot_description", urdf_string_);
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

bool BipedalWheelHW::setupTransmission(ros::NodeHandle & root_nh)
{
  if (!is_actuator_specified_) return true;
  try {
    transmission_loader_ = std::make_unique<transmission_interface::TransmissionInterfaceLoader>(
      this, &robot_transmissions_);
  } catch (const std::invalid_argument & ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  } catch (const pluginlib::LibraryLoadException & ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }
  // Perform transmission loading
  if (!transmission_loader_->load(urdf_string_)) {
    return false;
  }
  act_to_jnt_state_ =
    robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>();
  jnt_to_act_effort_ =
    robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>();
  auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
  std::vector<std::string> names = effort_joint_interface->getNames();
  for (const auto & name : names)
    effort_joint_handles_.push_back(effort_joint_interface->getHandle(name));

  return true;
}

bool BipedalWheelHW::setupJointLimit(ros::NodeHandle & root_nh)
{
  if (!is_actuator_specified_) return true;
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
  for (const auto & joint_handle : effort_joint_handles_) {
    bool has_joint_limits{}, has_soft_limits{};
    std::string name = joint_handle.getName();
    // Get limits from URDF
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_handle.getName());
    if (urdf_joint == nullptr) {
      ROS_ERROR_STREAM("URDF joint not found " << name);
      return false;
    }
    // Get limits from URDF
    if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
      has_joint_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has URDF position limits.");
    } else if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_DEBUG_STREAM("Joint " << name << " does not have a URDF limit.");
    // Get soft limits from URDF
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
      has_soft_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from URDF.");
    } else
      ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from URDF.");
    // Get limits from ROS param
    if (joint_limits_interface::getJointLimits(joint_handle.getName(), root_nh, joint_limits)) {
      has_joint_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has rosparam position limits.");
    }
    // Get soft limits from ROS param
    if (joint_limits_interface::getSoftJointLimits(joint_handle.getName(), root_nh, soft_limits)) {
      has_soft_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from ROS param.");
    } else
      ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from ROS param.");

    // Slightly reduce the joint limits to prevent floating point errors
    if (joint_limits.has_position_limits) {
      joint_limits.min_position += std::numeric_limits<double>::epsilon();
      joint_limits.max_position -= std::numeric_limits<double>::epsilon();
    }
    if (has_soft_limits) {  // Use soft limits
      ROS_DEBUG_STREAM("Using soft saturation limits");
      effort_jnt_soft_limits_interface_.registerHandle(
        joint_limits_interface::EffortJointSoftLimitsHandle(
          joint_handle, joint_limits, soft_limits));
    } else if (has_joint_limits) {
      ROS_DEBUG_STREAM("Using saturation limits (not soft limits)");
      effort_jnt_saturation_interface_.registerHandle(
        joint_limits_interface::EffortJointSaturationHandle(joint_handle, joint_limits));
    }
  }
  return true;
}

void BipedalWheelHW::publishActuatorState(const ros::Time & time)
{
  if (!is_actuator_specified_) return;
  if (last_publish_time_ + ros::Duration(1.0 / 100.0) < time) {
    if (actuator_state_pub_->trylock()) {
      bipedal_wheel_msgs::ActuatorState actuator_state;
      for (const auto & id2act_datas : bus_id2act_data_)
        for (const auto & act_data : id2act_datas.second) {
          actuator_state.stamp.push_back(act_data.second.stamp);
          actuator_state.name.push_back(act_data.second.name);
          actuator_state.type.push_back(act_data.second.type);
          actuator_state.bus.push_back(id2act_datas.first);
          actuator_state.id.push_back(act_data.first);
          actuator_state.halted.push_back(act_data.second.halted);
          actuator_state.need_calibration.push_back(act_data.second.need_calibration);
          actuator_state.calibrated.push_back(act_data.second.calibrated);
          actuator_state.calibration_reading.push_back(act_data.second.calibration_reading);
          actuator_state.position_raw.push_back(act_data.second.q_raw);
          actuator_state.velocity_raw.push_back(act_data.second.qd_raw);
          actuator_state.temperature.push_back(act_data.second.temp);
          actuator_state.circle.push_back(act_data.second.q_circle);
          actuator_state.last_position_raw.push_back(act_data.second.q_last);
          actuator_state.frequency.push_back(act_data.second.frequency);
          actuator_state.position.push_back(act_data.second.pos);
          actuator_state.velocity.push_back(act_data.second.vel);
          actuator_state.effort.push_back(act_data.second.effort);
          actuator_state.commanded_effort.push_back(act_data.second.cmd_effort);
          actuator_state.executed_effort.push_back(act_data.second.exe_effort);
        }
      actuator_state_pub_->msg_ = actuator_state;
      actuator_state_pub_->unlockAndPublish();
      last_publish_time_ = time;
    }
  }
}

BipedalWheelHW::~BipedalWheelHW()
{
  gpio_running_ = false;
  if (gpio_thread_.joinable()) gpio_thread_.join();
  for (auto bus : can_buses_) bus->disable();
}

}  // namespace bipedal_wheel_hw