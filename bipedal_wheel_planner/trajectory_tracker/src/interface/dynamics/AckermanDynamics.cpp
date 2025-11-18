//
// Created by guanlin on 25-9-28.
//

#include "trajectory_tracker/interface/dynamics/AckermanDynamics.h"

namespace trajectory_tracker
{
ocs2::vector_t AckermanDynamics::computeFlowMap(
  ocs2::scalar_t time, const ocs2::vector_t & state, const ocs2::vector_t & input,
  const ocs2::PreComputation &)
{
  // state x y phi
  // input v w
  ocs2::vector_t stateDerivative(STATE_DIM);
  stateDerivative(0) = input(0) * cos(state(2));
  stateDerivative(1) = input(0) * sin(state(2));
  stateDerivative(2) = input(1);

  return stateDerivative;
}

ocs2::VectorFunctionLinearApproximation AckermanDynamics::linearApproximation(
  ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u,
  const ocs2::PreComputation & preComp)
{
  ocs2::VectorFunctionLinearApproximation dynamics;
  dynamics.f = computeFlowMap(t, x, u, preComp);

  ocs2::matrix_t & dfdx = dynamics.dfdx;
  dfdx.setZero(STATE_DIM, STATE_DIM);
  dfdx(0, 2) = -u(0) * sin(x(2));
  dfdx(1, 2) = u(0) * cos(x(2));

  ocs2::matrix_t & dfdu = dynamics.dfdu;
  dfdu.setZero(STATE_DIM, INPUT_DIM);
  dfdu(0, 0) = cos(x(2));
  dfdu(1, 0) = sin(x(2));
  dfdu(2, 1) = 1;

  return dynamics;
}
}  // namespace trajectory_tracker
