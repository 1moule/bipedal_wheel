//
// Created by guanlin on 25-9-28.
//

#include "bipedal_wheel_planner/trajectory_tracker/interface/AckermanInterface.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <iostream>
#include <string>

#include "bipedal_wheel_planner/trajectory_tracker/interface/constraint/CollisionConstraint.h"
#include "bipedal_wheel_planner/trajectory_tracker/interface/cost/AckermanQuadraticTrackingCost.h"
#include "bipedal_wheel_planner/trajectory_tracker/interface/dynamics/AckermanDynamics.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace bipedal_wheel_planner {
using namespace ocs2;

AckermanInterface::AckermanInterface(const std::string &taskFile, const std::string &libraryFolder) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath))
    std::cerr << "[BipedalInterface] Loading task file: " << taskFilePath << std::endl;
  else
    throw std::invalid_argument("[BipedalInterface] Task file not found: " + taskFilePath.string());
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[BipedalInterface] Generated library path: " << libraryFolderPath << std::endl;

  // Default initial condition
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  // DDP SQP MPC settings
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp");

  // OptimalControlProblem
  setupOptimalControlProblem(taskFile, libraryFolder);
}

void AckermanInterface::setupOptimalControlProblem(const std::string &taskFile, const std::string &libraryFolder) {
  // Optimal control problem
  problemPtr_ = std::make_unique<OptimalControlProblem>();

  // Reference Manager
  referenceManagerPtr_ = std::make_shared<ReferenceManager>();

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  dynamicsPtr = std::make_unique<AckermanDynamics>();
  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  loadData::loadEigenMatrix(taskFile, "R", R);
  loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
  std::cerr << "Q:  \n" << Q << "\n";
  std::cerr << "R:  \n" << R << "\n";
  std::cerr << "Qf: \n" << Qf << "\n";
  problemPtr_->costPtr->add("cost", std::make_unique<AckermanStateInputQuadraticCost>(Q, R));
  problemPtr_->finalCostPtr->add("finalCost", std::make_unique<AckermanStateFinalQuadraticCost>(Qf));

  // Constraint
  std::unique_ptr<CollisionConstraint> collisionConstraintPtr = std::make_unique<CollisionConstraint>(gridMap_);
  ocs2::RelaxedBarrierPenalty::Config barrierCollisionPenaltyConfig(1e4, 0.2);
  problemPtr_->stateSoftConstraintPtr->add("CollisionConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(
      std::move(collisionConstraintPtr),
      std::make_unique<ocs2::RelaxedBarrierPenalty>(barrierCollisionPenaltyConfig))));

  // Rollout
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings);

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}
}  // namespace bipedal_wheel_planner