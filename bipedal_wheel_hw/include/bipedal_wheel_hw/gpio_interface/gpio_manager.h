//
// Created by yezi on 2021/9/9.
//

#pragma once

#include <XmlRpcValue.h>
#include <fcntl.h>
#include <poll.h>
#include <ros/ros.h>

#include <map>
#include <string>

#include "gpio_interface.h"

namespace bipedal_wheel_hw
{
class GpioManager
{
public:
  GpioManager() = default;
  ~GpioManager() = default;

  void setGpioDirection(hardware_interface::gpio::GpioData gpioData);
  void readGpio(std::vector<hardware_interface::gpio::GpioData> & gpio_state_values);
  void writeGpio(const std::vector<hardware_interface::gpio::GpioData> & gpio_command_values);
};
}  // namespace bipedal_wheel_hw