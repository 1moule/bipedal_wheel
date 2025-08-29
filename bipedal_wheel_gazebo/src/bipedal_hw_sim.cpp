//
// Created by guanlin on 25-8-29.
//

#include "bipedal_wheel_gazebo/bipedal_hw_sim.h"

#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

namespace bipedal_wheel_gazebo
{
bool BipedalHWSim::initSim(
  const std::string & robot_namespace, ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model, const urdf::Model * urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret =
    DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&imu_sensor_interface_);
  XmlRpc::XmlRpcValue xml_rpc_value;

  if (!model_nh.getParam("gazebo/imus", xml_rpc_value))
    ROS_WARN("No imu specified");
  else
    parseImu(xml_rpc_value, parent_model);
  world_ = parent_model->GetWorld();  // For gravity
  return ret;
}

void BipedalHWSim::readSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
  for (auto & imu : imu_datas_) {
    ignition::math::Pose3d pose = imu.link_ptr->WorldPose();
    imu.time_stamp = time;
    imu.ori[0] = pose.Rot().X();
    imu.ori[1] = pose.Rot().Y();
    imu.ori[2] = pose.Rot().Z();
    imu.ori[3] = pose.Rot().W();
    ignition::math::Vector3d rate = imu.link_ptr->RelativeAngularVel();
    imu.angular_vel[0] = rate.X();
    imu.angular_vel[1] = rate.Y();
    imu.angular_vel[2] = rate.Z();

    ignition::math::Vector3d gravity = {0., 0., -9.81};
    ignition::math::Vector3d accel =
      imu.link_ptr->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linear_acc[0] = accel.X();
    imu.linear_acc[1] = accel.Y();
    imu.linear_acc[2] = accel.Z();
  }
}

void BipedalHWSim::parseImu(
  XmlRpc::XmlRpcValue & imuDatas, const gazebo::physics::ModelPtr & parentModel)
{
  ROS_ASSERT(imuDatas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = imuDatas.begin(); it != imuDatas.end(); ++it) {
    if (!it->second.hasMember("frame_id")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    } else if (!it->second.hasMember("orientation_covariance_diagonal")) {
      ROS_ERROR_STREAM(
        "Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    } else if (!it->second.hasMember("angular_velocity_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    } else if (!it->second.hasMember("linear_acceleration_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }
    XmlRpc::XmlRpcValue oriCov = imuDatas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(oriCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(oriCov.size() == 3);
    for (int i = 0; i < oriCov.size(); ++i) {
      ROS_ASSERT(oriCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue angularCov = imuDatas[it->first]["angular_velocity_covariance"];
    ROS_ASSERT(angularCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angularCov.size() == 3);
    for (int i = 0; i < angularCov.size(); ++i) {
      ROS_ASSERT(angularCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue linearCov = imuDatas[it->first]["linear_acceleration_covariance"];
    ROS_ASSERT(linearCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(linearCov.size() == 3);
    for (int i = 0; i < linearCov.size(); ++i) {
      ROS_ASSERT(linearCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }

    std::string frameId = imuDatas[it->first]["frame_id"];
    gazebo::physics::LinkPtr link_ptr = parentModel->GetLink(frameId);
    ROS_ASSERT(link_ptr != nullptr);
    imu_datas_.push_back((ImuData{
      .link_ptr = link_ptr,
      .ori = {0., 0., 0., 0.},
      .ori_cov =
        {static_cast<double>(oriCov[0]), 0., 0., 0., static_cast<double>(oriCov[1]), 0., 0., 0.,
         static_cast<double>(oriCov[2])},
      .angular_vel = {0., 0., 0.},
      .angular_vel_cov =
        {static_cast<double>(angularCov[0]), 0., 0., 0., static_cast<double>(angularCov[1]), 0., 0.,
         0., static_cast<double>(angularCov[2])},
      .linear_acc = {0., 0., 0.},
      .linear_acc_cov = {
        static_cast<double>(linearCov[0]), 0., 0., 0., static_cast<double>(linearCov[1]), 0., 0.,
        0., static_cast<double>(linearCov[2])}}));
    ImuData & imu_data = imu_datas_.back();
    imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(
      it->first, frameId, imu_data.ori, imu_data.ori_cov, imu_data.angular_vel,
      imu_data.angular_vel_cov, imu_data.linear_acc, imu_data.linear_acc_cov));
  }
}
}  // namespace bipedal_wheel_gazebo
PLUGINLIB_EXPORT_CLASS(bipedal_wheel_gazebo::BipedalHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
