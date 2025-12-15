//
// Created by guanlin on 25-12-11.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>

#include <memory>

#include "bipedal_wheel_planner/trajectory_generator/perception_tools/GridMap.hpp"

namespace bipedal_wheel_planner
{
class CollisionConstraint final : public ocs2::StateConstraint
{
public:
  CollisionConstraint(std::shared_ptr<grid_map::GridMap> & gridMapPtr);

  ~CollisionConstraint() override = default;

  CollisionConstraint * clone() const override { return new CollisionConstraint(*this); }

  bool isActive(ocs2::scalar_t time) const override { return 1; }
  size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; }
  ocs2::vector_t getValue(
    ocs2::scalar_t time, const ocs2::vector_t & state,
    const ocs2::PreComputation & preComp) const override;

  ocs2::VectorFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time, const ocs2::vector_t & state,
    const ocs2::PreComputation & preComp) const override;

  std::shared_ptr<grid_map::GridMap> gridMapPtr_;

private:
  ocs2::scalar_t distance_threshold_ = 0.6;
};
}  // namespace bipedal_wheel_planner