//
// Created by guanlin on 25-9-28.
//

#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "bipedal_wheel_planner/trajectory_generator/perception_tools/GridMap.hpp"
#include "bipedal_wheel_planner/trajectory_tracker/interface/definitions.h"

namespace bipedal_wheel_planner
{
using namespace ocs2;

class AckermanInterface : public RobotInterface
{
public:
  AckermanInterface(const std::string & taskFile, const std::string & libraryFolder);
  ~AckermanInterface() = default;
  void setupOptimalControlProblem(const std::string & taskFile, const std::string & libraryFolder);
  void setupGridMap(std::shared_ptr<grid_map::GridMap> gridMap) { gridMap_ = gridMap; }

  const OptimalControlProblem & getOptimalControlProblem() const override { return *problemPtr_; }

  sqp::Settings & sqpSettings() { return sqpSettings_; }
  mpc::Settings & mpcSettings() { return mpcSettings_; }

  const vector_t & getInitialState() { return initialState_; }
  const RolloutBase & getRollout() const { return *rolloutPtr_; }

  const Initializer & getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override
  {
    return referenceManagerPtr_;
  }

private:
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  vector_t initialState_{STATE_DIM};

  std::shared_ptr<grid_map::GridMap> gridMap_;
};
}  // namespace bipedal_wheel_planner