//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_MODES__MODES_HPP_
#define GAITOS_MODES__MODES_HPP_

#include <string>
#include <vector>

#include "gaitos_core/Gains.hpp"
#include "gaitos_core/OperationModeBase.hpp"

namespace gaitos_modes
{

/**
 * @class ZeroTorqueMode
 * @brief Every joint passive (Unitree ZeroTorque / FSM 0). Controller: joint.
 */
class ZeroTorqueMode : public gaitos::OperationModeBase
{
public:
  void enter(const gaitos::RobotState & state) override {(void)state;}
  gaitos::ModeStatus update(const gaitos::RobotState & state, gaitos::ControlReference & reference) override;

protected:
  bool on_initialize() override {return true;}
};

/**
 * @class DampMode
 * @brief Joints damped, no stiffness (Unitree Damp / FSM 1): the safe mode, the robot
 * lowers slowly. Parameter `gains` (only kd is used; default kd 2.0).
 */
class DampMode : public gaitos::OperationModeBase
{
public:
  void enter(const gaitos::RobotState & state) override {(void)state;}
  gaitos::ModeStatus update(const gaitos::RobotState & state, gaitos::ControlReference & reference) override;

protected:
  bool on_initialize() override;

private:
  gaitos::JointGains gains_;
};

/**
 * @class PostureMode
 * @brief Moves the joints to one or several SRDF poses in sequence (Unitree StandUp,
 * StandDown, Sit, RecoveryStand, R1 Squat...). Each step interpolates (smoothstep) from the
 * previous pose (the measured joints for the first one) in `duration` s.
 *
 * @code{.yaml}
 * stand_up:
 *   plugin: gaitos_modes/PostureMode
 *   pose: stand            # or a sequence: poses: [lie, stand]
 *   duration: 1.2          #                durations: [1.0, 1.2]
 *   gains: {legs: {kp: 60.0, kd: 3.0}}
 *   next: balance_stand    # when finished (default: hold the last pose)
 * @endcode
 */
class PostureMode : public gaitos::OperationModeBase
{
public:
  void enter(const gaitos::RobotState & state) override;
  gaitos::ModeStatus update(const gaitos::RobotState & state, gaitos::ControlReference & reference) override;

  /// Progress of the sequence [0, 1]
  double progress() const;

protected:
  bool on_initialize() override;

private:
  struct Step
  {
    std::string pose;
    std::vector<double> q;
    double duration;
  };
  std::vector<Step> steps_;
  gaitos::JointGains gains_;
  std::vector<double> start_;
  double elapsed_{0.0};
  double last_time_{0.0};
};

/**
 * @class BalanceStandMode
 * @brief Standing, the body pose follows the pose goal (Unitree BalanceStand + Euler): the
 * position of the goal is the body offset over the nominal stance and its orientation the
 * body roll, pitch and yaw. The goal is relative to the body, so it changes the posture
 * from the current one. A twist command can switch to locomotion (`on_twist` parameter of
 * the mode manager). Controller: stance. Parameter `reset_pose_on_enter` (true).
 */
class BalanceStandMode : public gaitos::OperationModeBase
{
public:
  void enter(const gaitos::RobotState & state) override;
  gaitos::ModeStatus update(const gaitos::RobotState & state, gaitos::ControlReference & reference) override;
  bool accepts_pose_goal() const override {return true;}

protected:
  bool on_initialize() override;

private:
  bool reset_on_enter_{true};
  uint64_t goal_seq_{0};
  Eigen::Vector3d offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
};

/**
 * @class LocomotionMode
 * @brief Walking (Unitree Move, R1 Start / FSM 811). Follows the body twist command
 * (stops if none arrives for `command_timeout` s) or walks to a pose goal given relative
 * to the body when it was received: the goal is fixed in the odometry frame and tracked
 * with proportional gains `goal_gain` [vx, vy, wz], up to `goal_max_velocity`, until it is
 * within `goal_tolerance` [m, rad]. A newer twist cancels the goal. Needs a valid
 * estimate. Parameters: gait (name), body_offset, body_rpy. Controller: gait.
 */
class LocomotionMode : public gaitos::OperationModeBase
{
public:
  gaitos::CommandResult can_enter(const gaitos::RobotState & state) const override;
  void enter(const gaitos::RobotState & state) override;
  gaitos::ModeStatus update(const gaitos::RobotState & state, gaitos::ControlReference & reference) override;
  bool accepts_twist() const override {return true;}
  bool accepts_pose_goal() const override {return true;}

  bool has_goal() const {return has_goal_;}

protected:
  bool on_initialize() override;

private:
  double command_timeout_{0.5};
  Eigen::Vector3d goal_gain_{1.0, 1.0, 1.5};
  Eigen::Vector3d goal_max_{0.5, 0.3, 0.8};
  Eigen::Vector2d goal_tolerance_{0.05, 0.05};
  std::string gait_;
  Eigen::Vector3d body_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_rpy_{Eigen::Vector3d::Zero()};
  uint64_t goal_seq_{0};
  bool has_goal_{false};
  double goal_stamp_{0.0};
  Eigen::Vector3d goal_position_{Eigen::Vector3d::Zero()};   ///< Odometry frame
  double goal_yaw_{0.0};
};

}  // namespace gaitos_modes

#endif  // GAITOS_MODES__MODES_HPP_
