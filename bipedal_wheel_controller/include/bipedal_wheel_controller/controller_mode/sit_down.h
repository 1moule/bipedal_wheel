//
// Created by guanlin on 25-9-3.
//

#pragma once

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/definitions.h"

namespace bipedal_wheel_controller
{
class SitDown : public ModeBase
{
public:
  void execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period) override;
  const char* name() const override
  {
    return "SIT_DOWN";
  }
};
}  // namespace bipedal_wheel_controller
