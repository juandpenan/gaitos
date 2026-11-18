//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_SYSTEM__SAFETY_HPP_
#define GAITOS_SYSTEM__SAFETY_HPP_

#include <map>
#include <string>
#include <vector>

#include "gaitos_common/Params.hpp"
#include "gaitos_common/RobotState.hpp"
#include "gaitos_model/RobotModel.hpp"

namespace gaitos
{

/**
 * @class SafetyMonitor
 * @brief Joint limits checked on every cycle (outside the safe mode). A violation that lasts
 * `cycles` consecutive cycles sends the robot to the safe mode.
 *
 * @code{.yaml}
 * safety:
 *   limits:
 *     enabled: true
 *     position_margin: 0.1   # [rad] beyond the URDF limits (< 0: off)
 *     velocity_scale: 1.5    # times the URDF velocity limit (0: off)
 *     torque_scale: 1.5      # times the URDF effort limit, on the measured torque (0: off)
 *     cycles: 3
 * @endcode
 */
class SafetyMonitor
{
public:
  void configure(const Params & limits, RobotModelPtr model);

  /**
   * @return empty if everything is within limits, the reason otherwise
   */
  std::string check(const JointState & joints);

  void reset() {std::fill(count_.begin(), count_.end(), 0);}

private:
  RobotModelPtr model_;
  bool enabled_{true};
  double position_margin_{0.1};
  double velocity_scale_{1.5};
  double torque_scale_{1.5};
  int cycles_{3};
  std::vector<int> count_;
};

/**
 * @class Teleop
 * @brief Hand held remote (state.remote): emergency stop, modes on button combinations and
 * body twist from the sticks.
 *
 * @code{.yaml}
 * teleop:
 *   enabled: true
 *   estop: "L2+B"            # safe mode at once, from any mode
 *   buttons: ["L2+A", "START"]           # on press of buttons[i] ...
 *   modes: [stand_up, balance_stand]      # ... request modes[i]
 *   twist_scale: [0.5, 0.3, 1.0]   # left stick y -> vx, left x -> -vy, right x -> -wz
 *   deadzone: 0.1
 *   timeout: 0.5             # [s] remote data older than this is ignored
 * @endcode
 */
class Teleop
{
public:
  struct Actions
  {
    bool estop{false};
    std::string mode;            ///< Mode requested in this cycle (empty: none)
    bool twist{false};           ///< Send `linear` / `angular` this cycle
    Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
  };

  void configure(const Params & teleop);
  bool enabled() const {return enabled_;}
  Actions update(const RemoteInput & remote, double now);

private:
  bool enabled_{false};
  uint32_t estop_{0};
  std::vector<std::pair<uint32_t, std::string>> modes_;
  Eigen::Vector3d scale_{0.5, 0.3, 1.0};
  double deadzone_{0.1};
  double timeout_{0.5};
  uint32_t previous_{0};
  bool sticks_active_{false};
};

}  // namespace gaitos

#endif  // GAITOS_SYSTEM__SAFETY_HPP_
