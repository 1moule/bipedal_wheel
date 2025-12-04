//
// Created by guanlin on 25-9-17.
//

#include "bipedal_wheel_hw/can_interface/can_frame_codec.h"

namespace bipedal_wheel_hw
{
void CanFrameCodec::decodeRmMotor(const CanFrameStamp & frame_stamp, ActData & act_data)
{
  can_frame frame = frame_stamp.frame;
  act_data.q_raw = (frame.data[0] << 8u) | frame.data[1];
  act_data.qd_raw = (frame.data[2] << 8u) | frame.data[3];
  int16_t cur = (frame.data[4] << 8u) | frame.data[5];
  act_data.temp = frame.data[6];
  // Multiple circle
  if (act_data.seq != 0)  // not the first receive
  {
    if (act_data.q_raw - act_data.q_last > 4096)
      act_data.q_circle--;
    else if (act_data.q_raw - act_data.q_last < -4096)
      act_data.q_circle++;
  }
  try {  // Duration will be out of dual 32-bit range while motor failure
    act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
  } catch (std::runtime_error & ex) {
  }
  act_data.stamp = frame_stamp.stamp;
  act_data.seq++;
  act_data.q_last = act_data.q_raw;
  // Converter raw CAN data to position velocity and effort.
  auto type = getMotoTypeFromString(act_data.type);
  act_data.pos = getPositionFactorIntegerToRad(type) *
                 static_cast<double>(act_data.q_raw + 8191 * act_data.q_circle);
  act_data.vel = getVelocityFactorIntegerToRadPerSec(type) * static_cast<double>(act_data.qd_raw);
  act_data.effort = getTorqueFactorIntegerToNm(type) * static_cast<double>(cur);
  // Low pass filter
  act_data.lp_filter->input(act_data.vel, frame_stamp.stamp.toSec());
  act_data.vel = act_data.lp_filter->output();
}

void CanFrameCodec::decodeDmMotor(
  const bipedal_wheel_hw::CanFrameStamp & frame_stamp, bipedal_wheel_hw::ActData & act_data)
{
  can_frame frame = frame_stamp.frame;
  act_data.q_raw = (frame.data[1] << 8) | frame.data[2];
  uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
  uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];
  // Converter raw CAN data to position velocity and effort.
  auto type = getMotoTypeFromString(act_data.type);
  act_data.pos = fromRaw(act_data.q_raw, -getMaxPosition(type), getMaxPosition(type), 16);
  act_data.vel = fromRaw(qd, -getMaxVelocity(type), getMaxVelocity(type), 12);
  act_data.effort = fromRaw(cur, -getMaxTorque(type), getMaxTorque(type), 12);
  try {  // Duration will be out of dual 32-bit range while motor failure
    act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
  } catch (std::runtime_error & ex) {
  }
  act_data.stamp = frame_stamp.stamp;
  act_data.seq++;
  // Low pass filter
  act_data.lp_filter->input(act_data.vel, frame_stamp.stamp.toSec());
  act_data.vel = act_data.lp_filter->output();
}

void CanFrameCodec::decodeImuGyro(const CanFrameStamp & frame_stamp, ImuData & imu_data)
{
  can_frame frame = frame_stamp.frame;
  imu_data.gyro_updated = true;
  imu_data.angular_vel[0] =
    (((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.angular_vel_coeff) +
    imu_data.angular_vel_offset[0];
  imu_data.angular_vel[1] =
    (((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.angular_vel_coeff) +
    imu_data.angular_vel_offset[1];
  imu_data.angular_vel[2] =
    (((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.angular_vel_coeff) +
    imu_data.angular_vel_offset[2];
  imu_data.time_stamp = frame_stamp.stamp;
  auto temp = (int16_t)((frame.data[6] << 3) | (frame.data[7] >> 5));
  if (temp > 1023) temp -= 2048;
  imu_data.temperature = temp * imu_data.temp_coeff + imu_data.temp_offset;
}

void CanFrameCodec::decodeImuAccel(const CanFrameStamp & frame_stamp, ImuData & imu_data)
{
  can_frame frame = frame_stamp.frame;
  imu_data.accel_updated = true;
  imu_data.linear_acc[0] = ((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.accel_coeff;
  imu_data.linear_acc[1] = ((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.accel_coeff;
  imu_data.linear_acc[2] = ((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.accel_coeff;
  imu_data.time_stamp = frame_stamp.stamp;
  imu_data.camera_trigger = frame.data[6] & 1;
  imu_data.enabled_trigger = frame.data[6] & 2;
  double dt = (frame_stamp.stamp - last_imu_update_stamp_).toSec();
  if (dt > 0.1) dt = 0.0;  // prevent large dt
  last_imu_update_stamp_ = frame_stamp.stamp;
  imu_data.imu_filter->update(
    imu_data.linear_acc[0], imu_data.linear_acc[1], imu_data.linear_acc[2], imu_data.angular_vel[0],
    imu_data.angular_vel[1], imu_data.angular_vel[2], dt);
}

std::vector<can_frame> CanFrameCodec::encodeRmMotor(
  const std::pair<const int, ActData> & id2act_data)
{
  auto type = getMotoTypeFromString(id2act_data.second.type);
  int id = id2act_data.first - 0x201;
  double sign =
    (getTorqueFactorNmToInteger(type) * id2act_data.second.exe_effort < 0.0) ? -1.0 : 1.0;
  double cmd =
    sign *
    fmin(fabs(getTorqueFactorNmToInteger(type) * id2act_data.second.exe_effort), getMaxOut(type));

  can_frame rm_frame0{}, rm_frame1{};
  // safety first
  std::fill(std::begin(rm_frame0.data), std::end(rm_frame0.data), 0);
  std::fill(std::begin(rm_frame1.data), std::end(rm_frame1.data), 0);
  rm_frame0.can_id = 0x200;
  rm_frame0.can_dlc = 8;
  rm_frame1.can_id = 0x1FF;
  rm_frame1.can_dlc = 8;
  std::vector<can_frame> frames;
  if (-1 < id && id < 4) {
    rm_frame0.data[2 * id] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
    rm_frame0.data[2 * id + 1] = static_cast<uint8_t>(cmd);
    frames.push_back(rm_frame0);
  } else if (3 < id && id < 8) {
    rm_frame1.data[2 * (id - 4)] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
    rm_frame1.data[2 * (id - 4) + 1] = static_cast<uint8_t>(cmd);
    frames.push_back(rm_frame1);
  }
  return frames;
}

std::vector<can_frame> CanFrameCodec::encodeDmMotor(
  const std::pair<const int, ActData> & id2act_data)
{
  can_frame frame{};
  frame.can_id = id2act_data.first;
  frame.can_dlc = 8;
  auto type = getMotoTypeFromString(id2act_data.second.type);
  uint16_t q_des =
    toRaw<uint16_t>(id2act_data.second.cmd_pos, -getMaxPosition(type), getMaxPosition(type), 16);
  uint16_t qd_des =
    toRaw<uint16_t>(id2act_data.second.cmd_vel, -getMaxVelocity(type), getMaxVelocity(type), 12);
  uint16_t kp = toRaw<uint16_t>(id2act_data.second.cmd_kp, 0., getMaxKp(type), 12);
  uint16_t kd = toRaw<uint16_t>(id2act_data.second.cmd_kd, 0., getMaxKd(type), 12);
  uint16_t tau =
    toRaw<uint16_t>(id2act_data.second.exe_effort, -getMaxTorque(type), getMaxTorque(type), 12);
  frame.data[0] = q_des >> 8;
  frame.data[1] = q_des;
  frame.data[2] = qd_des >> 4;
  frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
  frame.data[4] = kp;
  frame.data[5] = kd >> 4;
  frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
  frame.data[7] = tau;
  std::vector<can_frame> frames;
  frames.push_back(frame);
  return frames;
}
}  // namespace bipedal_wheel_hw
