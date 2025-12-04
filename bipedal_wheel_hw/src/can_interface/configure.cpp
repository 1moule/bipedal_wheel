//
// Created by guanlin on 25-9-17.
//

#include "bipedal_wheel_hw/can_interface/can_frame_codec.h"

namespace bipedal_wheel_hw
{
/*****************************************************
 * rm motor
 *****************************************************/
uint16_t CanFrameCodec::getMaxOut(MotorType motorType)
{
  switch (motorType) {
    case MotorType::M3508:
      return 16384;
    case MotorType::M2006:
      return 10000;
    case MotorType::GM6020:
      return 30000;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getMaxOut] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getTorqueFactorIntegerToNm(MotorType motorType)
{
  switch (motorType) {
    case MotorType::M3508:
      return 20. / 16384. * 0.0156223893;
    case MotorType::M2006:
      return 10. / 10000. * 0.18;
    case MotorType::GM6020:
      return 5.880969e-5;  // special coefficient;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getTorqueFactorIntegerToNm] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getTorqueFactorNmToInteger(MotorType motorType)
{
  switch (motorType) {
    case MotorType::M3508:
    case MotorType::M2006:
      return 1. / getTorqueFactorIntegerToNm(motorType);
    case MotorType::GM6020:
      return 25000;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getTorqueFactorIntegerToNm] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getPositionFactorIntegerToRad(MotorType motorType)
{
  switch (motorType) {
    case MotorType::M3508:
      return 2. * M_PI / 8192.;
    case MotorType::M2006:
      return 2. * M_PI / 8192. / 36;
    case MotorType::GM6020:
      return 2. * M_PI / 8192.;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getPositionFactorIntegerToRad] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getVelocityFactorIntegerToRadPerSec(MotorType motorType)
{
  switch (motorType) {
    case MotorType::M3508:
      return 2. * M_PI / 60.;
    case MotorType::M2006:
      return 2. * M_PI / 60. / 36;
    case MotorType::GM6020:
      return 2. * M_PI / 60.;
    default:
      ROS_ERROR(
        "[bipedal_wheel_hw::CanFrameCodec::getVelocityFactorIntegerToRadPerSec] Unknown motor type");
      return 0;
  }
}

/*****************************************************
 * dm motor
 *****************************************************/
double CanFrameCodec::getMaxPosition(MotorType motorType)
{
  switch (motorType) {
    case MotorType::DM4310:
      return 12.5;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getMaxPosition] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getMaxVelocity(MotorType motorType)
{
  switch (motorType) {
    case MotorType::DM4310:
      return 30;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getMaxVelocity] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getMaxTorque(MotorType motorType)
{
  switch (motorType) {
    case MotorType::DM4310:
      return 10;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getMaxTorque] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getMaxKp(MotorType motorType)
{
  switch (motorType) {
    case MotorType::DM4310:
      return 500;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getMaxKp] Unknown motor type");
      return 0;
  }
}

double CanFrameCodec::getMaxKd(MotorType motorType)
{
  switch (motorType) {
    case MotorType::DM4310:
      return 5;
    default:
      ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getMaxKd] Unknown motor type");
      return 0;
  }
}

MotorType CanFrameCodec::getMotoTypeFromString(const std::string & motorTypeString)
{
  if (motorTypeString == "rm_3508") {
    return MotorType::M3508;
  } else if (motorTypeString == "rm_2006") {
    return MotorType::M2006;
  } else if (motorTypeString == "rm_6020") {
    return MotorType::GM6020;
  } else if (motorTypeString == "dm_4310")
    return MotorType::DM4310;
  else {
    ROS_ERROR("[bipedal_wheel_hw::CanFrameCodec::getMotoTypeFromString] Unknown motor type");
    return MotorType::M3508;
  }
}
}  // namespace bipedal_wheel_hw
