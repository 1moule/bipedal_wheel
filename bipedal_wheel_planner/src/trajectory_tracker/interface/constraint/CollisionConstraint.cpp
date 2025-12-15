//
// Created by guanlin on 25-12-11.
//

#include "bipedal_wheel_planner/trajectory_tracker/interface/constraint/CollisionConstraint.h"

#include "bipedal_wheel_planner/trajectory_tracker/interface/definitions.h"

namespace bipedal_wheel_planner
{

CollisionConstraint::CollisionConstraint(std::shared_ptr<grid_map::GridMap> & gridMapPtr)
: StateConstraint(ocs2::ConstraintOrder::Quadratic), gridMapPtr_(gridMapPtr)
{
}

ocs2::vector_t CollisionConstraint::getValue(
  ocs2::scalar_t time, const ocs2::vector_t & state, const ocs2::PreComputation & preComp) const
{
  if (!gridMapPtr_) {
    ocs2::vector_t constraint(1);
    constraint(0) = 1.0;
    return constraint;
  }
  Eigen::Vector2d currentPosition(state(0), state(1));
  double distance = gridMapPtr_->getDistance(currentPosition);
  ocs2::vector_t constraint(1);
  constraint(0) = distance - distance_threshold_;
  return constraint;
}

ocs2::VectorFunctionQuadraticApproximation CollisionConstraint::getQuadraticApproximation(
  ocs2::scalar_t time, const ocs2::vector_t & state, const ocs2::PreComputation & preComp) const
{
  ocs2::VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.dfdx.setZero(1, STATE_DIM);
  quadraticApproximation.dfdu.setZero(1, INPUT_DIM);
  quadraticApproximation.dfdxx.resize(1);
  quadraticApproximation.dfdxx[0].setZero(STATE_DIM, STATE_DIM);
  quadraticApproximation.dfduu.resize(1);
  quadraticApproximation.dfduu[0].setZero(INPUT_DIM, INPUT_DIM);
  quadraticApproximation.dfdux.resize(1);
  quadraticApproximation.dfdux[0].setZero(INPUT_DIM, STATE_DIM);

  if (!gridMapPtr_) {
    quadraticApproximation.f.resize(1);
    quadraticApproximation.f(0) = 1.0;
    return quadraticApproximation;
  }

  Eigen::Vector2d currentPosition(state(0), state(1));
  double distance{};
  Eigen::Vector2d gradient;

  gridMapPtr_->getDistanceAndGradient(currentPosition, distance, gradient);

  quadraticApproximation.f.resize(1);
  quadraticApproximation.f(0) = distance - distance_threshold_;

  quadraticApproximation.dfdx(0, 0) = gradient.x();
  quadraticApproximation.dfdx(0, 1) = gradient.y();

  quadraticApproximation.dfdxx[0](0, 1) = 2.0;
  quadraticApproximation.dfdxx[0](1, 1) = 2.0;

  return quadraticApproximation;
}
}  // namespace bipedal_wheel_planner
