//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_UNITREE_SIM__UNITREEGOEMULATOR_HPP_
#define GAITOS_UNITREE_SIM__UNITREEGOEMULATOR_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gaitos_unitree_sim/UnitreeEmulatorBase.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace gaitos_unitree_sim
{

/// Api ids of the Go2 sport service (unitree_sdk2 go2/sport/sport_api.hpp)
namespace go2
{
constexpr int64_t kDamp = 1001;
constexpr int64_t kBalanceStand = 1002;
constexpr int64_t kStopMove = 1003;
constexpr int64_t kStandUp = 1004;
constexpr int64_t kStandDown = 1005;
constexpr int64_t kRecoveryStand = 1006;
constexpr int64_t kEuler = 1007;
constexpr int64_t kMove = 1008;
constexpr int64_t kSit = 1009;
constexpr int64_t kRiseSit = 1010;
constexpr int64_t kSpeedLevel = 1015;
constexpr int64_t kSwitchJoystick = 1027;
constexpr int64_t kPose = 1028;
}  // namespace go2

/**
 * @class UnitreeGoEmulator
 * @brief The DDS interface of a Unitree Go2 (unitree_go) in front of a simulated gaitos, so
 * that a program written for the real robot (ROS 2 with the unitree_ros2 messages, the
 * SportClient protocol) runs unchanged against MuJoCo or Gazebo. gaitos plays the role of the
 * sport service: its modes stand for the sport modes.
 *
 * Publishes (absolute names, as the robot): lowstate and lf/lowstate (unitree_go/LowState:
 * motors in `motor_index` slots or the model order, IMU, foot forces, a full battery),
 * sportmodestate and lf/sportmodestate (unitree_go/SportModeState: the gaitos estimate, sport
 * mode, gait, feet in the body frame and their forces).
 *
 * Serves the sport service (api/sport/request|response, unitree_api; the SportClient JSON:
 * {"x", "y", "z"} for Euler and Move, {"data"} for the rest):
 *  - Damp 1001, BalanceStand 1002, StandUp 1004, StandDown 1005, RecoveryStand 1006, Sit 1009,
 *    RiseSit 1010: a gaitos mode each (`api.ids` / `api.modes`)
 *  - Move 1008: body twist (vx, vy, vyaw) kept for `move_timeout` s, only in `walking_modes`
 *    (from balance_stand it starts the gait); StopMove 1003 stops it
 *  - Euler 1007: body roll, pitch, yaw while standing (`pose_modes`)
 *  - SpeedLevel 1015, SwitchJoystick 1027, Pose 1028: accepted, no effect
 * Tricks and other gaits (Hello, Stretch, dances, flips, ClassicWalk...) answer 3203.
 * Not emulated: lowcmd, the wireless controller topic, the CRC of lowstate (0).
 */
class UnitreeGoEmulator : public UnitreeEmulatorBase
{
public:
  Reply handle_sport(int64_t api_id, const std::string & parameter);
  std::optional<int> sport_mode_of(const std::string & mode) const;
  void fill_lowstate(const gaitos::Snapshot & s, unitree_go::msg::LowState & msg) const;
  bool fill_sport_state(const gaitos::Snapshot & s, unitree_go::msg::SportModeState & msg) const;

protected:
  void configure_robot(const gaitos::Params & params) override;

private:
  std::vector<int> motor_of_joint_;
  std::vector<int> leg_of_slot_;   ///< Unitree foot order (FR, FL, RR, RL) -> gaitos leg
  std::map<int64_t, std::string> api_modes_;
  std::map<std::string, int> sport_modes_;
  std::vector<std::string> pose_modes_;
  double move_timeout_{1.0};

  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowstate_pub_;
  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lf_lowstate_pub_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sport_pub_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr lf_sport_pub_;
};

}  // namespace gaitos_unitree_sim

#endif  // GAITOS_UNITREE_SIM__UNITREEGOEMULATOR_HPP_
