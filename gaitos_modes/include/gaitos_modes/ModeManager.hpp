//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_MODES__MODEMANAGER_HPP_
#define GAITOS_MODES__MODEMANAGER_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gaitos_core/OperationModeBase.hpp"
#include "gaitos_core/PluginLoader.hpp"

namespace gaitos
{

/**
 * @class ModeManager
 * @brief Operation modes of the robot, without ROS: loads the mode plugins, validates the
 * requests (allowed_from, can_enter), runs the active mode, chains finished modes
 * (`next`) and applies the safety rules.
 *
 * @code{.yaml}
 * initial_mode: damp       # (ROS 2 parameters of the gaitos node)
 * safety:
 *   safe_mode: damp           # on failures, falls and lost estimate
 *   max_tilt: 1.0             # [rad] fall detection (0: off)
 *   fall_check_modes: []      # modes with fall detection (default: every mode but the safe
 *                             # mode, zero torque modes and modes with fall_check: false)
 * modes:
 *   damp: {plugin: gaitos_modes/DampMode}
 *   balance_stand: {plugin: gaitos_modes/BalanceStandMode, controller: stance,
 *                   on_twist: locomotion}
 * @endcode
 */
class ModeManager
{
public:
  using ChangeCallback = std::function<void(const std::string & from, const std::string & to,
      const std::string & reason)>;

  /**
   * @brief Load the modes
   * @param config whole configuration (modes, initial_mode, safety)
   * @throws std::runtime_error on errors
   */
  void configure(const Params & config, RobotModelPtr model);

  void set_change_callback(ChangeCallback cb) {callback_ = std::move(cb);}

  /**
   * @brief Request a mode
   */
  CommandResult request(const std::string & name, const RobotState & state);

  /**
   * @brief Go to the safe mode (always accepted)
   */
  void force_safe(const RobotState & state, const std::string & reason);

  /**
   * @brief Run the active mode: safety checks, automatic transitions and reference
   * @return name of the controller the active mode uses
   */
  const std::string & update(RobotState & state, ControlReference & reference);

  const std::string & current() const {return current_;}
  OperationModeBase * current_mode() const;
  std::vector<std::string> modes() const;
  const std::string & safe_mode() const {return safe_mode_;}
  bool has_mode(const std::string & name) const {return modes_.count(name) > 0;}

  /// Controllers referenced by the modes
  std::vector<std::string> controllers() const;

private:
  void switch_to(const std::string & name, const RobotState & state, const std::string & reason);

  struct Entry
  {
    std::shared_ptr<OperationModeBase> mode;
    std::string controller;
    std::string next;
    std::string on_twist;
    std::vector<std::string> allowed_from;
    bool fall_check{true};
  };

  PluginLoader<OperationModeBase> loader_{"gaitos::OperationModeBase"};   // before the modes
  std::map<std::string, Entry> modes_;
  std::string current_;
  std::string safe_mode_;
  double max_tilt_{1.0};
  double entered_{0.0};   ///< Time the current mode started [s]
  bool chain_failed_{false};   ///< The current mode could not chain to its next one
  ChangeCallback callback_;
  Logger logger_{"gaitos.modes"};
};

}  // namespace gaitos

#endif  // GAITOS_MODES__MODEMANAGER_HPP_
