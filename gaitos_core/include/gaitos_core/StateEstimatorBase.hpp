//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__STATEESTIMATORBASE_HPP_
#define GAITOS_CORE__STATEESTIMATORBASE_HPP_

#include "gaitos_common/RobotState.hpp"
#include "gaitos_core/PluginBase.hpp"

namespace gaitos
{

/**
 * @class StateEstimatorBase
 * @brief Estimation of the floating base (pose and twist, the odometry of the robot) from
 * the IMU, the leg kinematics, the contacts and the external odometries
 * (state.external_odometry).
 */
class StateEstimatorBase : public PluginBase
{
public:
  /**
   * @param state robot state (processed imu, joints, contacts, external odometries)
   * @param[out] estimate base state; set estimate.valid
   * @return true if estimate holds a new estimate
   */
  virtual bool update(const RobotState & state, BaseState & estimate) = 0;
};

}  // namespace gaitos

#endif  // GAITOS_CORE__STATEESTIMATORBASE_HPP_
