//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CONTROLLERS__CONTROLLERS_HPP_
#define GAITOS_CONTROLLERS__CONTROLLERS_HPP_

#include <map>
#include <string>
#include <vector>

#include "gaitos_controllers/LegIk.hpp"
#include "gaitos_core/ControllerBase.hpp"

namespace gaitos_controllers
{

/**
 * @class JointPdController
 * @brief Tracks the joint targets of the reference (positions, velocities, gains and
 * torques as given by the mode). Used by the posture, damp and zero torque modes.
 */
class JointPdController : public gaitos::ControllerBase
{
public:
  bool update(
    const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
    gaitos::JointCommand & command) override;

protected:
  bool on_initialize() override {return true;}
};

/**
 * @class StanceController
 * @brief Standing with all the feet on the ground. The body moves over the feet following
 * the reference body_offset and body_rpy (limited by `max_offset` and `max_rpy`, and in
 * rate by `max_speed` and `max_angular_speed`); the feet stay at the nominal stance of the
 * `stance_pose` SRDF pose. The rest of joints hold that pose. Optional gravity
 * compensation (`gravity_compensation`, 0..1) and attitude feedback (`attitude_gain`).
 * On activation the targets are blended from the current joints in `blend_time` s.
 */
class StanceController : public gaitos::ControllerBase
{
public:
  void activate(const gaitos::RobotState & state) override;
  bool update(
    const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
    gaitos::JointCommand & command) override;

protected:
  bool on_initialize() override;

private:
  LegIk ik_;
  gaitos::JointGains gains_;
  Eigen::Vector3d max_offset_{0.05, 0.05, 0.1};
  Eigen::Vector3d max_rpy_{0.3, 0.3, 0.4};
  double max_speed_{0.2};
  double max_angular_speed_{0.6};
  double gravity_{0.0};
  double attitude_gain_{0.0};
  double blend_time_{0.3};
  Eigen::Vector3d offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  std::vector<double> start_q_;
  std::vector<double> q_;
  double elapsed_{0.0};
};

/**
 * @struct GaitPattern
 * @brief Periodic gait: every leg cycles stance -> swing; `offsets` is the phase of each
 * leg (leg order of the model), `duty` the stance fraction of the period.
 */
struct GaitPattern
{
  double period{0.4};
  double duty{0.6};
  double step_height{0.08};
  double sway{0.0};   ///< [m] lateral body shift towards the stance feet (bipeds)
  std::vector<double> offsets;
};

/**
 * @class GaitController
 * @brief Kinematic gait for any number of legs (trot for quadrupeds, alternating steps for
 * bipeds), following the body twist of the reference.
 *
 * Feet in stance move against the commanded body velocity; feet in swing follow a
 * cycloid to the touchdown point given by the Raibert heuristic (nominal foot + v T_st / 2
 * + `raibert_gain` (v - v_cmd)). The body pose reference (offset, rpy) is applied over the
 * planned feet. Joints come from inverse kinematics, with velocity feedforward and the
 * gravity compensation of the stance legs. When the command is zero for `idle_time` the
 * legs stop at their next touchdown and the robot stands; a new command starts the gait.
 *
 * The body velocity measured by the estimator closes a PI loop on (vx, vy, wz)
 * (`velocity_feedback.kp`, `.ki`, `.max_integral`), which compensates the slip and the
 * compliance of the stance legs of a purely kinematic gait.
 *
 * Bipeds need `sway`: the body shifts sideways over the stance foot, peaking at the
 * middle of the swing of the other foot (amplitude in m, ramped in and out). The yaw of each
 * foot is planned too (a planted foot turns against the body, a swinging one to its
 * touchdown yaw): bipeds with oriented feet turn with it. Legs sharing a joint (NAO
 * HipYawPitch) are solved together by LegIk.
 *
 * Parameters: stance_pose, default_gait, gaits.<name>.{period, duty, step_height, offsets,
 * sway}, max_sway_speed, attitude_gain (roll and pitch feedback on the body pose),
 * max_velocity [vx, vy, wz], max_acceleration [ax, ay, alpha], raibert_gain,
 * velocity_feedback.{kp, ki, max_integral} ([vx, vy, wz] each),
 * gravity_compensation, gains.stance, gains.swing (gain blocks for the legs),
 * gains.upper_body, idle_time, blend_time.
 */
class GaitController : public gaitos::ControllerBase
{
public:
  void activate(const gaitos::RobotState & state) override;
  bool update(
    const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
    gaitos::JointCommand & command) override;

  /// Legs in stance (tests, monitoring)
  const std::vector<bool> & stance() const {return stance_;}
  bool walking() const {return walking_;}

protected:
  bool on_initialize() override;

private:
  const GaitPattern & pattern(const std::string & name) const;

  LegIk ik_;
  std::map<std::string, GaitPattern> gaits_;
  std::string default_gait_{"trot"};
  Eigen::Vector3d max_velocity_{0.8, 0.4, 1.0};
  Eigen::Vector3d max_acceleration_{1.5, 1.0, 3.0};
  double raibert_gain_{0.03};
  Eigen::Vector3d feedback_kp_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d feedback_ki_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d feedback_max_{0.3, 0.2, 0.8};
  double gravity_{0.0};
  double idle_time_{0.3};
  double blend_time_{0.3};
  double max_sway_speed_{0.3};
  double attitude_gain_{0.0};
  gaitos::JointGains stance_gains_;
  gaitos::JointGains swing_gains_;
  gaitos::JointGains upper_gains_;

  // Runtime
  Eigen::Vector3d cmd_{Eigen::Vector3d::Zero()};   ///< Smoothed (vx, vy, wz)
  Eigen::Vector3d integral_{Eigen::Vector3d::Zero()};   ///< Velocity feedback integral
  Eigen::Vector2d sway_{Eigen::Vector2d::Zero()};       ///< Current body shift (level frame)
  double phase_time_{0.0};
  double idle_{0.0};
  bool walking_{false};
  bool stopping_{false};
  std::vector<bool> stance_;
  std::vector<bool> frozen_;
  std::vector<Eigen::Vector3d> foot_;        ///< Planned feet, level body frame
  std::vector<Eigen::Vector3d> liftoff_;
  std::vector<double> foot_yaw_;        ///< Planned yaw of each foot (oriented IK: bipeds)
  std::vector<double> liftoff_yaw_;
  std::vector<double> q_;
  std::vector<double> start_q_;
  double elapsed_{0.0};
};

}  // namespace gaitos_controllers

#endif  // GAITOS_CONTROLLERS__CONTROLLERS_HPP_
