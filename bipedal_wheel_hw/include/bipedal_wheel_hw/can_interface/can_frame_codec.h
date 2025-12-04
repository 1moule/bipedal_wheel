//
// Created by guanlin on 25-9-17.
//

#pragma once

#include "bipedal_wheel_hw/can_interface/definitions.h"

namespace bipedal_wheel_hw
{
enum class MotorType { M3508, M2006, GM6020, DM4310, DM8009};

class CanFrameCodec
{
public:
  CanFrameCodec() = default;
  ~CanFrameCodec() = default;
  void decodeRmMotor(const CanFrameStamp & frame_stamp, ActData & act_data);
  void decodeDmMotor(const CanFrameStamp & frame_stamp, ActData & act_data);
  void decodeImuGyro(const CanFrameStamp & frame_stamp, ImuData & imu_data);
  void decodeImuAccel(const CanFrameStamp & frame_stamp, ImuData & imu_data);
  std::vector<can_frame> encodeRmMotor(const std::pair<const int, ActData> & id2act_data);
  std::vector<can_frame> encodeDmMotor(const std::pair<const int, ActData> & id2act_data);

private:
  static uint16_t getMaxOut(MotorType motorType);
  static double getTorqueFactorIntegerToNm(MotorType motorType);
  static double getTorqueFactorNmToInteger(MotorType motorType);
  static double getPositionFactorIntegerToRad(MotorType motorType);
  static double getVelocityFactorIntegerToRadPerSec(MotorType motorType);
  static double getMaxVelocity(MotorType motorType);
  static double getMaxTorque(MotorType motorType);
  static double getMaxKp(MotorType motorType);
  static double getMaxKd(MotorType motorType);
  static double getMaxPosition(MotorType motorType);
  static MotorType getMotoTypeFromString(const std::string & motorTypeString);

  template <typename T>
  double fromRaw(T value, double xMin, double xMax, int bits) const
  {
    double span = xMax - xMin;
    double offset = xMin;
    return ((double)value) * span / ((double)((1 << bits) - 1)) + offset;
  }
  template <typename T>
  T toRaw(double value, double xMin, double xMax, int bits) const
  {
    double span = xMax - xMin;
    double offset = xMin;
    return static_cast<T>(((value - offset) * ((1 << bits) - 1) / span));
  }

  ros::Time last_imu_update_stamp_{};
};
}  // namespace bipedal_wheel_hw
