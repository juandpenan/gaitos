//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_UNITREE_SIM__UNITREEHGEMULATOR_HPP_
#define GAITOS_UNITREE_SIM__UNITREEHGEMULATOR_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gaitos_unitree_sim/UnitreeEmulatorBase.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_hg/msg/bms_state.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace gaitos_unitree_sim
{

/// Api ids of the R1 services (loco service "sport", arm service "arm")
namespace hg
{
constexpr int64_t kGetFsmId = 7001;
constexpr int64_t kSetFsmId = 7101;
constexpr int64_t kSetVelocity = 7105;
constexpr int64_t kSetSpeedMode = 7107;
constexpr int64_t kExecuteAction = 7106;
constexpr int64_t kGetActionList = 7107;
constexpr int64_t kExecuteCustomAction = 7108;
constexpr int64_t kStopCustomAction = 7113;
}  // namespace hg

/**
 * @class UnitreeHgEmulator
 * @brief The DDS interface of a Unitree humanoid (R1, unitree_hg) in front of a simulated
 * gaitos, so that a ROS 2 stack written for the real robot (e.g. unitree_r1: r1_state,
 * r1_locomotion, r1_gesture) runs unchanged against MuJoCo or Gazebo. gaitos plays the role of
 * Unitree's controller: its modes stand for the FSM of the locomotion service.
 *
 * Publishes (absolute names, as the robot): lowstate (unitree_hg/LowState: motors in the
 * documented slots of `motor_index`, IMU, tick, mode_machine), secondary_imu, lf/bmsstate (a
 * full battery), odommodestate (unitree_go/SportModeState from the gaitos estimate),
 * arm/action/state (JSON: nothing holding).
 *
 * Serves:
 *  - sport: GetFsmId 7001 (gaitos mode -> FSM id, `fsm.report_*`), SetFsmId 7101 (FSM id ->
 *    gaitos mode, `fsm.ids` / `fsm.modes`), SetVelocity 7105 ({"velocity": [vx, vy, wz],
 *    "duration": s}, only in `walking_modes`), SetSpeedMode 7107 (accepted, no effect)
 *  - arm: GetActionList 7107 (empty), StopCustomAction 7113 (ok); the actions answer 3203
 * arm_sdk (`arm_sdk.enabled`, default false): the upper body commands of `/arm_sdk`
 * (unitree_hg/LowCmd: q, dq, tau, kp, kd of the motor slots, weight in mode_pr 0..100) go
 * over the walking controller with Gaitos::set_joint_commands(), only in the walking modes
 * (an R1 in damp or zero torque ignores arm_sdk). While commands with a weight above 0 arrive
 * the robot is "held": GetFsmId reports `arm_sdk.held_fsm` (816), SetVelocity is refused with
 * `arm_sdk.velocity_code` (127) and SetFsmId(811) with `arm_sdk.fsm_code` (1001). These
 * codes are what an R1 answered in tests (2026-11), not documented values. The joints must be
 * in `external_joints.groups` of gaitos (the SRDF group arm_sdk).
 *
 * Not emulated: the Dex3 hands, the CRC of lowstate (0).
 */
class UnitreeHgEmulator : public UnitreeEmulatorBase
{
public:
  std::optional<int> fsm_of_mode(const std::string & mode) const;
  std::optional<std::string> mode_of_fsm(int fsm) const;
  Reply handle_sport(int64_t api_id, const std::string & parameter);
  Reply handle_arm(int64_t api_id, const std::string & parameter);
  void fill_lowstate(const gaitos::Snapshot & s, unitree_hg::msg::LowState & msg) const;
  bool fill_odometry(const gaitos::Snapshot & s, unitree_go::msg::SportModeState & msg) const;
  /// arm_sdk command -> joint commands for gaitos, one per arm_sdk joint, with the weight of
  /// mode_pr (also 0: the release ramp of a client ends with it); empty outside the walking
  /// modes or when arm_sdk is disabled. Updates held().
  std::vector<gaitos::JointCommandOverride> arm_sdk_commands(const unitree_hg::msg::LowCmd & msg);
  /// The arms are held through arm_sdk: in a walking mode, and a command with weight > 0
  /// arrived less than `arm_sdk.timeout` ago
  bool held() const;

protected:
  void configure_robot(const gaitos::Params & params) override;

private:
  std::vector<int> motor_of_joint_;
  uint8_t mode_machine_{0};
  std::vector<std::pair<int, std::string>> fsm_requests_;   ///< FSM id -> mode to request
  std::map<std::string, int> fsm_reports_;                  ///< mode -> FSM id reported

  // arm_sdk
  std::vector<std::size_t> arm_sdk_joints_;   ///< model joints driven through arm_sdk
  double arm_sdk_timeout_{0.5};
  int held_fsm_{816};
  int32_t held_velocity_code_{127};
  int32_t held_fsm_code_{1001};
  std::atomic<double> last_held_{-1.0};   ///< node time of the last command with weight > 0
  rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr arm_sdk_sub_;


  rclcpp::Publisher<unitree_hg::msg::LowState>::SharedPtr lowstate_pub_;
  rclcpp::Publisher<unitree_hg::msg::IMUState>::SharedPtr imu_pub_;
  rclcpp::Publisher<unitree_hg::msg::BmsState>::SharedPtr bms_pub_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_state_pub_;
};

}  // namespace gaitos_unitree_sim

#endif  // GAITOS_UNITREE_SIM__UNITREEHGEMULATOR_HPP_
