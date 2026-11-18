//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__OPERATIONMODEBASE_HPP_
#define GAITOS_CORE__OPERATIONMODEBASE_HPP_

#include <string>
#include <vector>

#include "gaitos_common/RobotState.hpp"
#include "gaitos_core/PluginBase.hpp"

namespace gaitos
{

enum class ModeStatus
{
  RUNNING,    ///< Keep running
  FINISHED,   ///< Done: go to the next mode (`next` parameter) if any, else stay
  FAILED,     ///< Go to the safe mode
};

/**
 * @class OperationModeBase
 * @brief Operation mode of the robot (damp, stand up, sit, balance stand, locomotion...).
 *
 * A mode decides the controller to use (`controller` parameter) and fills its
 * reference each cycle. Common parameters, read by the ModeManager:
 *  - controller: name of the controller instance (default "joint")
 *  - next: mode to switch to when this one finishes
 *  - allowed_from: modes this one can be requested from (empty: any)
 */
class OperationModeBase : public PluginBase
{
public:
  /**
   * @brief Whether the mode can start now (e.g. locomotion needs a valid estimate)
   */
  virtual CommandResult can_enter(const RobotState & state) const
  {
    (void)state;
    return CommandResult::ok();
  }

  /**
   * @brief The mode starts
   */
  virtual void enter(const RobotState & state) = 0;

  /**
   * @brief Fill the reference of the controller
   */
  virtual ModeStatus update(const RobotState & state, ControlReference & reference) = 0;

  /**
   * @brief The mode ends
   */
  virtual void exit(const RobotState & state) {(void)state;}

  /// Uses state.twist_command
  virtual bool accepts_twist() const {return false;}
  /// Uses state.pose_goal
  virtual bool accepts_pose_goal() const {return false;}

  std::string controller() const {return params().get<std::string>("controller", "joint");}
  std::string next() const {return params().get<std::string>("next", "");}
  std::vector<std::string> allowed_from() const
  {
    return params().get<std::vector<std::string>>("allowed_from", {});
  }
};

}  // namespace gaitos

#endif  // GAITOS_CORE__OPERATIONMODEBASE_HPP_
