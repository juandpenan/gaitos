//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_DYNAMICS__DYNAMICCONTROLLERS_HPP_
#define GAITOS_DYNAMICS__DYNAMICCONTROLLERS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "gaitos_controllers/LegIk.hpp"
#include "gaitos_core/ControllerBase.hpp"
#include "gaitos_core/Gains.hpp"
#include "gaitos_dynamics/ConvexMpc.hpp"
#include "gaitos_dynamics/LipmPreview.hpp"

namespace gaitos_dynamics
{

/**
 * @class LipmZmpController
 * @brief Biped walking with ZMP preview control of the linear inverted pendulum (Kajita).
 *
 * A footstep planner turns the body twist into footsteps (step_time, double support,
 * step width, alternating feet); the ZMP reference goes from foot to foot in double
 * support and stays on the support foot in single support; the preview controller
 * produces the CoM trajectory that realizes it. The swing foot follows a cycloid, the body
 * yaw is the mean of the feet, and the joints come from the 6D inverse kinematics of the
 * legs (feet flat). Standing still, the ZMP stays between the feet.
 *
 * Everything is planned in a frame fixed at activation (position controlled robots, like
 * the R1). Optional feedback: attitude (`attitude_gain`, roll and pitch on the body) and
 * CoM from the estimator (`com_feedback.position`, `com_feedback.velocity`).
 *
 * The ZMP goes to the center of the sole (`foot_center` [x, y, z] in the foot frame: the
 * middle of the sole, e.g. ahead of and below the ankle frame).
 *
 * Feedforward (`feedforward`, 0..1): the ground reaction of the LIPM, F = m (c''x, c''y, g),
 * applied at the center of pressure (the ZMP in single support, the soles in double
 * support, shared by the distance to the ZMP), becomes joint torques tau = -J^T [F; M] of
 * each stance leg, M being its moment about the foot frame. Position controlled joints then
 * do not have to yield to carry the robot.
 *
 * Stabilizer (`dcm_feedback`, gain k): the divergent component of motion of the estimate,
 * xi = c + c'/omega, is compared with the planned one and moves the center of pressure of
 * the feedforward, p = p_plan + k (xi - xi_plan), clamped to the sole (`sole_half_size`
 * [x, y]): the ankle torques push the CoM back to the plan (ankle strategy). The position
 * part of the error is limited (`dcm_position_limit`, 0.03 m): the plan is open loop and
 * drifts from the robot (shorter or slipping steps), and that drift must not saturate the
 * ankles.
 *
 * Walking closes two loops with the estimator, since slipping feet make the open loop plan
 * drift: heading (`heading_gain`: the next footsteps turn to recover the planned body yaw;
 * `heading_body_gain`: the body also turns over the stance foot at once, limited to 0.2 rad)
 * and body velocity
 * (`velocity_feedback.ki`, `.max_integral` [vx, vy, wz], as in the GaitController).
 *
 * Parameters: stance_pose, step_time (0.6), double_support (0.2, fraction of a step),
 * step_height (0.04), step_width (nominal), start_delay (0.6), preview_time (1.6),
 * zmp_weight (1.0), jerk_weight (1e-6), dt (cycle time, 0.002), max_velocity [vx, vy, wz],
 * max_acceleration, idle_time, attitude_gain, gains, com_feedback.{position, velocity}.
 */
class LipmZmpController : public gaitos::ControllerBase
{
public:
  void activate(const gaitos::RobotState & state) override;
  bool update(
    const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
    gaitos::JointCommand & command) override;

  bool walking() const {return walking_;}
  const LipmPreview & preview() const {return preview_;}
  /// ZMP reference now and CoM plan, frame fixed at activation (tests, logging)
  Eigen::Vector2d zmp_reference() const {return zmp_ref_now_;}

protected:
  bool on_initialize() override;

private:
  struct Foot
  {
    Eigen::Vector2d p{Eigen::Vector2d::Zero()};
    double yaw{0.0};
  };
  struct Step
  {
    double t0{0.0};
    int support{0};          ///< leg index; -1: final step (feet together, ZMP to the middle)
    Foot support_foot;
    Foot swing_from;
    Foot swing_to;
    Eigen::Vector2d zmp_from{Eigen::Vector2d::Zero()};
    bool zero{false};        ///< No advance: the feet end side by side
  };

  Eigen::Vector2d zmp_at(double t) const;
  Eigen::Vector2d sole(const Foot & f) const;
  void replan(const Eigen::Vector3d & cmd);
  Foot place(const Foot & path, int leg) const;
  Eigen::Vector2d middle() const;

  gaitos_controllers::LegIk ik_;
  LipmPreview preview_;
  gaitos::JointGains gains_;
  double step_time_{0.6};
  double double_support_{0.2};
  double step_height_{0.04};
  double step_width_{0.0};
  double start_delay_{0.6};
  double idle_time_{0.5};
  double attitude_gain_{0.0};
  double com_fb_p_{0.0};
  double com_fb_v_{0.0};
  double dt_{0.002};
  Eigen::Vector2d foot_center_{Eigen::Vector2d::Zero()};
  double sole_depth_{0.0};      ///< Height of the sole in the foot frame (negative: below)
  double feedforward_{1.0};
  double heading_gain_{0.0};
  double heading_body_gain_{0.0};
  double heading_error_{0.0};
  double dcm_gain_{0.0};
  double dcm_position_limit_{0.03};
  Eigen::Vector2d sole_half_{0.08, 0.03};
  Eigen::Vector3d feedback_ki_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d feedback_max_{0.1, 0.1, 0.3};
  Eigen::Vector3d integral_{Eigen::Vector3d::Zero()};
  double mass_{0.0};
  Eigen::Vector3d max_velocity_{0.3, 0.15, 0.5};
  Eigen::Vector3d max_acceleration_{0.3, 0.3, 1.0};
  Eigen::Vector3d com_offset_{Eigen::Vector3d::Zero()};   ///< CoM in the base frame (stance pose)
  double ground_z_{0.0};
  std::array<int, 2> side_{{0, 1}};                       ///< left, right leg indices
  std::vector<Eigen::Matrix3d> foot_rotation_;            ///< Nominal foot orientation in the base

  // Runtime
  double t_{0.0};
  bool walking_{false};
  bool stopping_{false};
  double idle_{0.0};
  Eigen::Vector3d cmd_{Eigen::Vector3d::Zero()};
  std::vector<Foot> feet_;
  std::deque<Step> steps_;
  int next_support_{1};
  std::vector<double> q_;
  std::vector<double> start_q_;
  std::vector<Eigen::Vector2d> zmp_buffer_;
  Eigen::Vector2d zmp_ref_now_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d stand_point_{Eigen::Vector2d::Zero()};   ///< ZMP while standing
  double plan_yaw_{0.0};   ///< Planned body yaw of the previous cycle (plan frame)
  Eigen::Isometry3d odom_from_plan_{Eigen::Isometry3d::Identity()};
};

/**
 * @class ConvexMpcController
 * @brief Quadruped locomotion with convex MPC of the ground reaction forces (Di Carlo et
 * al. 2018). The stance legs apply the forces of the MPC as joint torques
 * (tau = -J^T R^T f); the swing legs follow a cycloid to the Raibert footstep with joint PD
 * through inverse kinematics. The gait schedule (gaits, offsets, period, duty) is the one of
 * the GaitController; standing still, every foot is in contact and the MPC holds the body
 * pose. Needs a valid state estimate.
 *
 * Parameters: stance_pose, horizon (10), mpc_dt (0.03), mu (0.5), fz_max (150),
 * weights (13: rpy, p, omega, v, g), force_weight, foot_height (height of the foot frame
 * over the ground), gaits, default_gait, max_velocity, max_acceleration, raibert_gain,
 * idle_time, gains.stance, gains.swing, gains.standing (stopped; default: gains.stance), gains.upper_body.
 */
class ConvexMpcController : public gaitos::ControllerBase
{
public:
  void activate(const gaitos::RobotState & state) override;
  bool update(
    const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
    gaitos::JointCommand & command) override;

  const std::vector<Eigen::Vector3d> & forces() const {return forces_;}
  const OsqpSolver::Result & last_result() const {return result_;}
  bool walking() const {return walking_;}

protected:
  bool on_initialize() override;

private:
  struct Gait
  {
    double period{0.4};
    double duty{0.6};
    double step_height{0.08};
    std::vector<double> offsets;
  };
  bool stance_at(std::size_t leg, double t, const Gait & g) const;

  gaitos_controllers::LegIk ik_;
  ConvexMpc mpc_;
  std::map<std::string, Gait> gaits_;
  std::string default_gait_{"trot"};
  double mpc_dt_{0.03};
  double foot_height_{0.0};
  double raibert_gain_{0.03};
  double idle_time_{0.3};
  Eigen::Vector3d max_velocity_{0.8, 0.4, 1.0};
  Eigen::Vector3d max_acceleration_{1.5, 1.0, 3.0};
  Eigen::Vector3d com_offset_{Eigen::Vector3d::Zero()};
  double com_height_{0.3};
  gaitos::JointGains stance_gains_;
  gaitos::JointGains swing_gains_;
  gaitos::JointGains standing_gains_;
  gaitos::JointGains upper_gains_;

  // Runtime
  Eigen::Vector3d cmd_{Eigen::Vector3d::Zero()};
  std::vector<double> hold_q_;   ///< Posture held while standing
  double phase_time_{0.0};
  double idle_{0.0};
  bool walking_{false};
  bool stopping_{false};
  std::vector<bool> frozen_;
  std::vector<bool> stance_;
  std::vector<Eigen::Vector3d> liftoff_;   ///< World
  std::vector<Eigen::Vector3d> forces_;    ///< World
  Eigen::Vector3d p_des_{Eigen::Vector3d::Zero()};
  double yaw_des_{0.0};
  double since_solve_{1e9};
  OsqpSolver::Result result_;
  std::vector<double> q_;
};

}  // namespace gaitos_dynamics

#endif  // GAITOS_DYNAMICS__DYNAMICCONTROLLERS_HPP_
