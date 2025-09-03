//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <ros/time.h>

namespace bipedal_wheel_controller
{

class BipedalController;

class ModeBase
{
public:
  virtual void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) = 0;
  virtual const char* name() const = 0;
  virtual ~ModeBase() = default;
};

}  // namespace bipedal_wheel_controller
