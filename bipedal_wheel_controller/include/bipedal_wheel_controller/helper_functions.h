//
// Created by guanlin on 25-8-27.
//

#pragma once

#include <angles/angles.h>
#include <cstddef>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Quaternion.h>
#include <hardware_interface/joint_command_interface.h>

#include "bipedal_wheel_controller/dynamics/gen_A.h"
#include "bipedal_wheel_controller/dynamics/gen_B.h"
#include "bipedal_wheel_controller/vmc/leg_conv_fwd.h"
#include "bipedal_wheel_controller/vmc/leg_conv.h"
#include "bipedal_wheel_controller/definitions.h"

namespace bipedal_wheel_controller
{
/**
 * Generate continuous-time state space matrices A and B
 * @param model_params
 * @param a
 * @param b
 * @param leg_length
 */
inline void generateAB(const std::unique_ptr<ModelParams>& model_params, Eigen::Matrix<double, STATE_DIM, STATE_DIM>& a,
                       Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& b, double leg_length)
{
  double A[36] = { 0. }, B[12]{ 0. };
  double L = leg_length * model_params->L_weight;
  double Lm = leg_length * model_params->Lm_weight;
  gen_A(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
        model_params->g, model_params->l, model_params->m_p, model_params->m_w, A);
  gen_B(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
        model_params->l, model_params->m_p, model_params->m_w, B);

  // clang-format off
  a<< 0.  ,1.,0.,0.,0.   ,0.,
      A[1],0.,0.,0.,A[25],0.,
      0.  ,0.,0.,1.,0.   ,0.,
      A[3],0.,0.,0.,A[27],0.,
      0.  ,0.,0.,0.,0.   ,1.,
      A[5],0.,0.,0.,A[29],0.;
  b<< 0.  ,0.  ,
      B[1],B[7],
      0.  ,0.  ,
      B[3],B[9],
      0.  ,0.  ,
      B[5],B[11];
  // clang-format on
}

/**
 * Fit the LQR gain matrix K as a cubic polynomial of leg length
 * @param Ks
 * @param L0s
 * @param coeffs
 */
inline void polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
                    Eigen::Matrix<double, 4, 12>& coeffs)
{
  int N = L0s.size();
  Eigen::MatrixXd A(N, 4), B(N, 12);
  for (int i = 0; i < N; ++i)
  {
    A.block(i, 0, 1, 4) << pow(L0s[i], 3), pow(L0s[i], 2), L0s[i], 1.0;
    Eigen::Map<const Eigen::Matrix<double, 12, 1>> flat(Ks[i].data());
    B.row(i) = flat.transpose();
  }
  coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * B);
}

/**
 * Calculate the normal support force
 * @param F
 * @param Tp
 * @param leg_length
 * @param acc_z
 * @param x
 * @param u
 * @param model_params
 * @return
 */
inline double calculateSupportForce(double F, double Tp, double leg_length, double acc_z,
                                    Eigen::Matrix<double, STATE_DIM, 1> x, Eigen::Matrix<double, CONTROL_DIM, 1> u,
                                    const std::unique_ptr<ModelParams>& model_params)
{
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a;
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b;
  generateAB(model_params, a, b, leg_length);

  double P = F * cos(x(0)) + Tp * sin(x(0)) / leg_length;
  double ddot_zM = acc_z - model_params->g;
  auto ddot_x = a * x + b * u;
  double ddot_theta = ddot_x(1);
  double ddot_zw = ddot_zM - leg_length * cos(x(0)) + 2 * leg_length * x(1) * sin(x(0)) +
                   +leg_length * (ddot_theta * sin(x(0)) + x(1) * x(1) * cos(x(0)));
  double Fn = model_params->m_w * ddot_zw + model_params->m_w * model_params->g + P;
  return Fn;
}

/**
 * Detect whether the leg is unstick
 * @param hip_effort
 * @param knee_effort
 * @param wheel_effort
 * @param hip_angle
 * @param knee_angle
 * @param leg_length
 * @param acc_z
 * @param model_params
 * @param x
 * @return
 */
inline bool unstickDetection(const double& hip_effort, const double& knee_effort, const double& wheel_effort,
                             const double& hip_angle, const double& knee_angle, const double& leg_length,
                             const double& acc_z, const std::unique_ptr<ModelParams>& model_params,
                             Eigen::Matrix<double, STATE_DIM, 1> x)
{
  double leg_F[2];
  leg_conv_fwd(hip_effort, knee_effort, hip_angle, knee_angle, leg_F);
  Eigen::Matrix<double, CONTROL_DIM, 1> u_left_real;
  u_left_real << wheel_effort, leg_F[1];
  double Fn = calculateSupportForce(leg_F[0], leg_F[1], leg_length, acc_z, x, u_left_real, model_params);
  if (Fn < 20.)
    return true;
  else
    return false;
}

/**
 * Detect the leg state before stand up: UNDER, FRONT, BEHIND
 * @param x
 * @param leg_state
 */
inline void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, int& leg_state)
{
  if (x[0] > -M_PI / 2 + 0.1 && x[0] < M_PI / 2 - 0.2)
    leg_state = LegState::UNDER;
  else if (x[0] < -M_PI / 2 + 0.1 && x[0] > -M_PI)
    leg_state = LegState::FRONT;
  else if (x[0] > M_PI / 2 - 0.2 && x[0] < M_PI)
    leg_state = LegState::BEHIND;
}

/**
 * Set up the desired leg motion during stand up
 * @param x
 * @param other_leg_state
 * @param leg_length
 * @param leg_theta
 * @param leg_state
 * @param theta_des
 * @param length_des
 */
inline void setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const int& other_leg_state,
                           const double& leg_length, const double& leg_theta, int& leg_state, double& theta_des,
                           double& length_des)
{
  switch (leg_state)
  {
    case LegState::UNDER:
      theta_des = 0.15;
      length_des = 0.05;
      break;
    case LegState::FRONT:
      theta_des = M_PI / 2 + 0.2;
      length_des = 0.4;
      if (abs(angles::shortest_angular_distance(x[0], M_PI / 2)) < 0.2 && abs(x[4]) < 0.1)
        leg_state = LegState::BEHIND;
      break;
    case LegState::BEHIND:
      theta_des = leg_theta;
      length_des = leg_length;
      if (other_leg_state != LegState::FRONT)
      {
        theta_des = 0.;
        length_des = 0.05;
      }
      break;
  }
}

/**
 * Compute the leg command using PID controllers
 * @param desired_length
 * @param desired_angle
 * @param current_length
 * @param current_angle
 * @param length_pid
 * @param angle_pid
 * @param leg_angle
 * @param period
 * @return
 */
inline LegCommand computePidLegCommand(double desired_length, double desired_angle, double current_length,
                                       double current_angle, control_toolbox::Pid& length_pid,
                                       control_toolbox::Pid& angle_pid, const double* leg_angle,
                                       const ros::Duration& period)
{
  LegCommand cmd;
  cmd.force = length_pid.computeCommand(desired_length - current_length, period);
  cmd.torque = -angle_pid.computeCommand(-angles::shortest_angular_distance(desired_angle, current_angle), period);
  leg_conv(cmd.force, cmd.torque, leg_angle[0], leg_angle[1], cmd.input);
  return cmd;
}

/**
 * Set joint commands to the joint handles
 * @param joints
 * @param left_cmd
 * @param right_cmd
 * @param wheel_left
 * @param wheel_right
 */
inline void setJointCommands(std::vector<hardware_interface::JointHandle*>& joints, const LegCommand& left_cmd,
                             const LegCommand& right_cmd, double wheel_left = 0., double wheel_right = 0.)
{
  if (joints.size() != 6)
    throw std::runtime_error("Joint handle vector size must be 6!");

  joints[0]->setCommand(left_cmd.input[0]);
  joints[1]->setCommand(left_cmd.input[1]);
  joints[2]->setCommand(right_cmd.input[0]);
  joints[3]->setCommand(right_cmd.input[1]);
  joints[4]->setCommand(wheel_left);
  joints[5]->setCommand(wheel_right);
}

/**
 * Convert quaternion to roll, pitch, yaw
 * @param q
 * @param roll
 * @param pitch
 * @param yaw
 */
inline void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
  yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  pitch = std::asin(as);
  roll = std::atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

}  // namespace bipedal_wheel_controller
