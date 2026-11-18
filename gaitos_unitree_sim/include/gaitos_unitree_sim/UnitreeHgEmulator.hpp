//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_UNITREE_SIM__UNITREEHGEMULATOR_HPP_
#define GAITOS_UNITREE_SIM__UNITREEHGEMULATOR_HPP_

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
 * Not emulated: arm_sdk, the Dex3 hands, the CRC of lowstate (0).
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

protected:
  void configure_robot(const gaitos::Params & params) override;

private:
  std::vector<int> motor_of_joint_;
  uint8_t mode_machine_{0};
  std::vector<std::pair<int, std::string>> fsm_requests_;   ///< FSM id -> mode to request
  std::map<std::string, int> fsm_reports_;                  ///< mode -> FSM id reported

  rclcpp::Publisher<unitree_hg::msg::LowState>::SharedPtr lowstate_pub_;
  rclcpp::Publisher<unitree_hg::msg::IMUState>::SharedPtr imu_pub_;
  rclcpp::Publisher<unitree_hg::msg::BmsState>::SharedPtr bms_pub_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_state_pub_;
};

}  // namespace gaitos_unitree_sim

#endif  // GAITOS_UNITREE_SIM__UNITREEHGEMULATOR_HPP_
