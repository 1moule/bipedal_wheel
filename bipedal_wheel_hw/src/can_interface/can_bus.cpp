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
// Created by qiayuan on 12/28/20.
//
#include "bipedal_wheel_hw/can_interface/can_bus.h"

#include <ros/ros.h>

#include <string>

namespace bipedal_wheel_hw
{
CanBus::CanBus(const std::string & bus_name, CanDataPtr data_ptr, int thread_priority)
: bus_name_(bus_name), data_ptr_(data_ptr)
{
  // Initialize device at can_device, false for no loop back.
  while (
    !socket_can_.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) &&
    ros::ok())
    ros::Duration(.5).sleep();

  ROS_INFO("Successfully connected to %s.", bus_name.c_str());

  can_frame_codec_ = std::make_unique<CanFrameCodec>();
}

void CanBus::enable()
{
  for (auto & item : *data_ptr_.id2act_data_) {
    if (item.second.type.find("dm") != std::string::npos) {
      for (int count = 0; count < 5; ++count) {
        can_frame frame{};
        for (int i = 0; i < 7; i++) {
          frame.data[i] = 0xFF;
        }
        frame.data[7] = 0xFC;
        frame.can_id = item.first;
        frame.can_dlc = 8;
        socket_can_.write(&frame);
      }
    }
  }
}

void CanBus::disable()
{
  for (auto & item : *data_ptr_.id2act_data_) {
    if (item.second.type.find("dm") != std::string::npos) {
      can_frame frame{};
      for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
      }
      frame.data[7] = 0xFD;
      frame.can_id = item.first;
      frame.can_dlc = 8;
      socket_can_.write(&frame);
    }
  }
}

void CanBus::write()
{
  for (auto & item : *data_ptr_.id2act_data_) {
    if (item.second.halted) continue;
    if (item.second.type.find("rm") != std::string::npos) {
      std::vector<can_frame> frames = can_frame_codec_->encodeRmMotor(item);
      if (frames.size() != 0) {
        for (auto & frame : frames) socket_can_.write(&frame);
      }
    } else if (item.second.type.find("dm") != std::string::npos) {
      std::vector<can_frame> frames = can_frame_codec_->encodeDmMotor(item);
      if (frames.size() != 0) {
        for (auto & frame : frames) socket_can_.write(&frame);
      }
    }
  }
}

void CanBus::read(ros::Time time)
{
  std::lock_guard<std::mutex> guard(mutex_);

  for (auto & imu : *data_ptr_.id2imu_data_) {
    imu.second.gyro_updated = false;
    imu.second.accel_updated = false;
  }

  for (const auto & frame_stamp : read_buffer_) {
    can_frame frame = frame_stamp.frame;
    // Check if  rm motor
    if (data_ptr_.id2act_data_->find(frame.can_id) != data_ptr_.id2act_data_->end()) {
      ActData & act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
      if ((frame_stamp.stamp - act_data.stamp).toSec() < 0.0005) continue;
      if (act_data.type.find("rm") != std::string::npos) {
        can_frame_codec_->decodeRmMotor(frame_stamp, act_data);
        continue;
      }
    }
    // Check if dm motor
    else if (
      data_ptr_.id2act_data_->find((frame.data[0] & 0xFF)) != data_ptr_.id2act_data_->end()) {
      ActData & act_data = data_ptr_.id2act_data_->find(frame.data[0] & 0xFF)->second;
      if ((frame_stamp.stamp - act_data.stamp).toSec() < 0.0005) continue;
      if (act_data.type.find("dm") != std::string::npos) {
        can_frame_codec_->decodeDmMotor(frame_stamp, act_data);
        continue;
      }
    }
    // Check if IMU gyro
    else if (data_ptr_.id2imu_data_->find(frame.can_id) != data_ptr_.id2imu_data_->end()) {
      ImuData & imu_data = data_ptr_.id2imu_data_->find(frame.can_id)->second;
      can_frame_codec_->decodeImuGyro(frame_stamp, imu_data);
      continue;
    }
    // Check if IMU accel
    else if (data_ptr_.id2imu_data_->find(frame.can_id - 1) != data_ptr_.id2imu_data_->end()) {
      ImuData & imu_data = data_ptr_.id2imu_data_->find(frame.can_id - 1)->second;
      can_frame_codec_->decodeImuAccel(frame_stamp, imu_data);
      continue;
    }
    if (frame.can_id != 0x0)
      ROS_ERROR_STREAM_ONCE(
        "Can not find defined device, id: 0x" << std::hex << frame.can_id
                                              << " on bus: " << bus_name_);
  }
  read_buffer_.clear();
}

void CanBus::write(can_frame * frame) { socket_can_.write(frame); }

void CanBus::frameCallback(const can_frame & frame)
{
  std::lock_guard<std::mutex> guard(mutex_);
  CanFrameStamp can_frame_stamp{.frame = frame, .stamp = ros::Time::now()};
  read_buffer_.push_back(can_frame_stamp);
}

}  // namespace bipedal_wheel_hw
