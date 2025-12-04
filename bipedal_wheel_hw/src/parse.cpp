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
// Created by qiayuan on 5/16/21.
//

#include <joint_limits_interface/joint_limits_urdf.h>
#include <transmission_interface/transmission_interface_loader.h>

#include "bipedal_wheel_common/lp_filter.h"
#include "bipedal_wheel_common/ros_tools.h"
#include "bipedal_wheel_hw/bipedal_wheel_hw.h"

namespace bipedal_wheel_hw
{
// Lots of ugly parse xml code...

bool BipedalWheelHW::parseActData(XmlRpc::XmlRpcValue & act_datas, ros::NodeHandle & robot_hw_nh)
{
  ROS_ASSERT(act_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try {
    for (auto it = act_datas.begin(); it != act_datas.end(); ++it) {
      // Get configuration
      const auto & name = it->first;
      const auto & data = it->second;
      if (!(data.hasMember("bus") && data.hasMember("type") && data.hasMember("id"))) {
        ROS_ERROR_STREAM(
          "Actuator " << name << " missing field(s): " << (!data.hasMember("bus") ? "bus " : "")
                      << (!data.hasMember("type") ? "type " : "")
                      << (!data.hasMember("id") ? "id " : ""));
        continue;
      }
      bool need_calibration =
        data.hasMember("need_calibration") && static_cast<bool>(data["need_calibration"]);
      if (!data.hasMember("need_calibration"))
        ROS_DEBUG_STREAM("Actuator " << name << " set no need calibration by default.");
      std::string bus = static_cast<std::string>(data["bus"]);
      std::string type = static_cast<std::string>(data["type"]);
      int id = static_cast<int>(data["id"]);
      // Create uncreated bus
      if (bus_id2act_data_.find(bus) == bus_id2act_data_.end())
        bus_id2act_data_.insert(std::make_pair(bus, std::unordered_map<int, ActData>()));
      // Create actuator on the bus
      if (!(bus_id2act_data_[bus].find(id) == bus_id2act_data_[bus].end())) {
        ROS_ERROR_STREAM("Repeat actuator on bus " << bus << " and ID " << id);
        return false;
      } else {
        ros::NodeHandle nh = ros::NodeHandle(robot_hw_nh, "actuators/" + it->first);
        bus_id2act_data_[bus].insert(std::make_pair(
          id, ActData{
                .name = it->first,
                .type = type,
                .stamp = ros::Time::now(),
                .need_calibration = need_calibration,
                .lp_filter = std::make_unique<LowPassFilter>(100.)}));
      }

      // for ros_control interface
      hardware_interface::ActuatorStateHandle act_state(
        bus_id2act_data_[bus][id].name, &bus_id2act_data_[bus][id].pos,
        &bus_id2act_data_[bus][id].vel, &bus_id2act_data_[bus][id].effort);
      act_state_interface_.registerHandle(act_state);
      // RoboMaster motors are effect actuator
      if (type.find("rm") != std::string::npos || type.find("dm") != std::string::npos) {
        effort_act_interface_.registerHandle(
          hardware_interface::ActuatorHandle(act_state, &bus_id2act_data_[bus][id].exe_effort));
      } else {
        ROS_ERROR_STREAM(
          "Actuator " << it->first
                      << "'s type neither RoboMaster(rm_xxx) nor Cheetah(cheetah_xxx)");
        return false;
      }
    }
    registerInterface(&act_state_interface_);
    registerInterface(&effort_act_interface_);
    is_actuator_specified_ = true;
  } catch (XmlRpc::XmlRpcException & e) {
    ROS_FATAL_STREAM(
      "Exception raised by XmlRpc while reading the "
      << "configuration: " << e.getMessage() << ".\n"
      << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}

bool BipedalWheelHW::parseGpioData(XmlRpc::XmlRpcValue & gpio_datas, ros::NodeHandle & robot_hw_nh)
{
  for (auto it = gpio_datas.begin(); it != gpio_datas.end(); ++it) {
    if (it->second.hasMember("pin")) {
      hardware_interface::gpio::GpioData gpio_data;
      if (std::string(gpio_datas[it->first]["direction"]) == "in") {
        gpio_data.type = hardware_interface::gpio::INPUT;
      } else if (std::string(gpio_datas[it->first]["direction"]) == "out") {
        gpio_data.type = hardware_interface::gpio::OUTPUT;
      } else {
        ROS_ERROR("Type set error of %s!", it->first.data());
        continue;
      }
      gpio_data.name = it->first;
      gpio_data.pin = gpio_datas[it->first]["pin"];
      gpio_data.value = new bool(false);
      gpio_manager_->setGpioDirection(gpio_data);
      gpio_state_values_.push_back(gpio_data);
      hardware_interface::gpio::GpioStateHandle gpio_state_handle(
        it->first, gpio_data.type, gpio_state_values_.back().value);
      gpio_state_interface_.registerHandle(gpio_state_handle);

      if (gpio_data.type == hardware_interface::gpio::OUTPUT) {
        gpio_command_values_.push_back(gpio_data);
        hardware_interface::gpio::GpioCommandHandle gpio_command_handle(
          it->first, gpio_data.type, gpio_command_values_.back().value);
        gpio_command_interface_.registerHandle(gpio_command_handle);
      }
    } else {
      ROS_ERROR("Module %s hasn't set pin ID", it->first.data());
    }
  }
  return true;
}
}  // namespace bipedal_wheel_hw