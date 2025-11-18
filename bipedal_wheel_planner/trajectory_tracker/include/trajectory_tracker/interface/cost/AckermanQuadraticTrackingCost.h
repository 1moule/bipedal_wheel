//
// Created by guanlin on 25-9-28.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

namespace trajectory_tracker
{
using namespace ocs2;

class AckermanStateInputQuadraticCost final : public QuadraticStateInputCost
{
public:
  AckermanStateInputQuadraticCost(matrix_t Q, matrix_t R)
  : QuadraticStateInputCost(std::move(Q), std::move(R))
  {
  }

  ~AckermanStateInputQuadraticCost() override = default;
  AckermanStateInputQuadraticCost * clone() const override
  {
    return new AckermanStateInputQuadraticCost(*this);
  }

private:
  std::pair<vector_t, vector_t> getStateInputDeviation(
    scalar_t time, const vector_t & state, const vector_t & input,
    const TargetTrajectories & targetTrajectories) const override
  {
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    return {state - xNominal, input};
  }
};

class AckermanStateFinalQuadraticCost final : public QuadraticStateCost
{
public:
  AckermanStateFinalQuadraticCost(matrix_t Q) : QuadraticStateCost(std::move(Q)) {}

  ~AckermanStateFinalQuadraticCost() override = default;
  AckermanStateFinalQuadraticCost * clone() const override
  {
    return new AckermanStateFinalQuadraticCost(*this);
  }

private:
  vector_t getStateDeviation(
    scalar_t time, const vector_t & state,
    const TargetTrajectories & targetTrajectories) const override
  {
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    return state - xNominal;
  }
};
}  // namespace trajectory_tracker
