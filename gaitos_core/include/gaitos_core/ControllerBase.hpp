//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__CONTROLLERBASE_HPP_
#define GAITOS_CORE__CONTROLLERBASE_HPP_

#include "gaitos_common/RobotState.hpp"
#include "gaitos_core/PluginBase.hpp"

namespace gaitos
{

/**
 * @class ControllerBase
 * @brief Whole-body controller: turns the reference of the active operation mode into
 * joint commands. Several controllers are loaded (e.g. joint, stance, gait) and each
 * operation mode selects the one it uses.
 */
class ControllerBase : public PluginBase
{
public:
  /**
   * @brief Called when it becomes the active controller (a mode that uses it starts)
   */
  virtual void activate(const RobotState & state) {(void)state;}

  /**
   * @brief Compute the joint commands for one cycle
   * @param state robot state
   * @param reference what the mode asks for
   * @param dt time since the previous cycle [s]
   * @param[out] command command of every joint (already sized)
   * @return false if no valid command could be computed
   */
  virtual bool update(
    const RobotState & state, const ControlReference & reference, double dt,
    JointCommand & command) = 0;
};

}  // namespace gaitos

#endif  // GAITOS_CORE__CONTROLLERBASE_HPP_
