//
// Created by guanlin on 25-9-28.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include "trajectory_tracker/interface/definitions.h"

namespace trajectory_tracker
{
using namespace ocs2;

class AckermanDynamics final : public SystemDynamicsBase
{
public:
  AckermanDynamics() = default;

  /** Destructor */
  ~AckermanDynamics() override = default;

  AckermanDynamics * clone() const override { return new AckermanDynamics(*this); }

  ocs2::vector_t computeFlowMap(
    ocs2::scalar_t time, const ocs2::vector_t & state, const ocs2::vector_t & input,
    const ocs2::PreComputation &) override;

  ocs2::VectorFunctionLinearApproximation linearApproximation(
    ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u,
    const ocs2::PreComputation & preComp) override;
};
}  // namespace trajectory_tracker
