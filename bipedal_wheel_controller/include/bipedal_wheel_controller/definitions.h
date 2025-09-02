//
// Created by guanlin on 25-8-30.
//

#pragma once

#include <array>
#include <utility>

namespace bipedal_wheel_controller
{
struct ModelParams
{
  double L_weight;   // Length weight to wheel axis
  double Lm_weight;  // Length weight to mass center
  double l;          // Leg rest length
  double m_w;        // Wheel mass
  double m_p;        // Leg mass
  double M;          // Body mass
  double i_w;        // Wheel inertia
  double i_p;        // Leg inertia
  double i_m;        // Body inertia
  double r;          // Wheel radius
  double g;          // Gravity acceleration
};

struct LegCommand
{
  double force;     // Thrust
  double torque;    // Torque
  double input[2];  // input
};

enum LegState
{
  UNDER,
  FRONT,
  BEHIND
};

enum JumpPhase
{
  SQUAT,
  JUMP,
  SHRINK,
  DONE
};

constexpr std::array<std::pair<JumpPhase, const double>, 3> jumpLengthDes = {
  { { JumpPhase::SQUAT, 0.15 }, { JumpPhase::JUMP, 0.4 }, { JumpPhase::SHRINK, 0.15 } }
};

constexpr static const int STATE_DIM = 6;
constexpr static const int CONTROL_DIM = 2;

}  // namespace bipedal_wheel_controller
