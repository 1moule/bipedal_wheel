//
// Created by guanlin on 25-9-15.
//

#pragma once

#include <linux/can.h>
#include <ros/ros.h>
#include <bipedal_wheel_common/complementary_filter.h>
#include <bipedal_wheel_common/lp_filter.h>

#include <string>
#include <unordered_map>

namespace bipedal_wheel_hw
{
typedef enum {
  NONE = 0x07,
  OVER_VOLTAGE,
  UNDER_VOLTAGE,
  OVER_CURRENT,
  MOS_OVER_TEMP,
  MOTOR_COIL_OVER_TEMP,
  COM_LOST,
  OVER_LOAD,
} DmError;

typedef enum {
  MIT,
  POS,
  VEL,
  EFFORT,
} ControlMode;

struct ActData
{
  std::string name;
  std::string type;
  ros::Time stamp;
  uint64_t seq = 0;
  DmError error;
  ControlMode mode;
  bool halted = false, need_calibration = false, calibrated = false, calibration_reading = false;
  uint16_t q_raw = 0;
  int16_t qd_raw = 0;
  uint8_t temp = 0;
  int64_t q_circle = 0;
  uint16_t q_last = 0;
  double frequency = 0.;
  double pos = 0., vel = 0., effort = 0.;
  double cmd_pos = 0., cmd_vel = 0., cmd_effort = 0., cmd_kp = 0., cmd_kd = 0., exe_effort = 0.;
  std::unique_ptr<LowPassFilter> lp_filter;
};

struct ImuData
{
  ros::Time time_stamp;
  std::string imu_name;
  double ori[4];
  double angular_vel[3], linear_acc[3];
  double angular_vel_offset[3];
  double ori_cov[9], angular_vel_cov[9], linear_acc_cov[9];
  double temperature, angular_vel_coeff, accel_coeff, temp_coeff, temp_offset;
  bool accel_updated, gyro_updated, camera_trigger;
  bool enabled_trigger;
  std::unique_ptr<imu_tools::ComplementaryFilter> imu_filter;
};

struct CanDataPtr
{
  std::unordered_map<int, ActData> * id2act_data_;
  std::unordered_map<int, ImuData> * id2imu_data_;
};

struct CanFrameStamp
{
  can_frame frame;
  ros::Time stamp;
};
}  // namespace bipedal_wheel_hw
