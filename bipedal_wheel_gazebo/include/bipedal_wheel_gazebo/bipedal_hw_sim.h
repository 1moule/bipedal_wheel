//
// Created by guanlin on 25-8-29.
//

#pragma once

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/imu_sensor_interface.h>

namespace bipedal_wheel_gazebo
{
struct ImuData
{
  gazebo::physics::LinkPtr link_ptr;
  ros::Time time_stamp;
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class BipedalHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:
  bool initSim(
    const std::string & robot_namespace, ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model, const urdf::Model * urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;

private:
  void parseImu(XmlRpc::XmlRpcValue & imuDatas, const gazebo::physics::ModelPtr & parentModel);

  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  gazebo::physics::WorldPtr world_;
  std::list<ImuData> imu_datas_;
  ros::ServiceServer switch_imu_service_;
};
}  // namespace bipedal_wheel_gazebo
