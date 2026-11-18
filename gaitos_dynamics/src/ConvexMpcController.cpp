//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_dynamics/DynamicControllers.hpp"

#include <algorithm>
#include <cmath>

#include "Common.hpp"
#include "gaitos_common/Math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_dynamics
{

using gaitos::Params;

using namespace detail;

// ==========================================================================================
// ConvexMpcController

bool ConvexMpcController::on_initialize()
{
  const auto legs = model().leg_count();
  if (legs == 0) {
    return false;
  }
  ik_.configure(model_ptr(), params().get<std::string>("stance_pose", "stand"));
  const Params gaits = params().sub("gaits");
  for (const auto & name : gaits.keys()) {
    const Params g = gaits.sub(name);
    Gait gait;
    gait.period = g.get<double>("period", gait.period);
    gait.duty = g.get<double>("duty", gait.duty);
    gait.step_height = g.get<double>("step_height", gait.step_height);
    gait.offsets = g.get<std::vector<double>>("offsets", {});
    if (gait.offsets.size() != legs) {
      logger().error() << "Gait " << name << " needs " << legs << " offsets";
      return false;
    }
    gaits_[name] = gait;
  }
  if (gaits_.empty()) {
    logger().error() << "No gaits configured";
    return false;
  }
  default_gait_ = params().get<std::string>("default_gait", gaits_.begin()->first);
  mpc_dt_ = params().get<double>("mpc_dt", mpc_dt_);
  foot_height_ = params().get<double>("foot_height", foot_height_);
  raibert_gain_ = params().get<double>("raibert_gain", raibert_gain_);
  idle_time_ = params().get<double>("idle_time", idle_time_);
  max_velocity_ = params().get_vector3("max_velocity", max_velocity_);
  max_acceleration_ = params().get_vector3("max_acceleration", max_acceleration_);
  stance_gains_ = gaitos::read_gains(params().sub("gains.stance"), model(), 0.0, 1.0);
  standing_gains_ = params().has("gains.standing") ?
    gaitos::read_gains(params().sub("gains.standing"), model(), 0.0, 1.0) : stance_gains_;
  swing_gains_ = gaitos::read_gains(params().sub("gains.swing"), model(), 40.0, 1.5);
  upper_gains_ = gaitos::read_gains(params().sub("gains.upper_body"), model(), 40.0, 1.0);

  const auto rb = model().tree().rigid_body(ik_.stance_q(), model().base_link());
  com_offset_ = rb.com;
  double feet_z = 0.0;
  for (const auto & f : ik_.nominal_feet()) {
    feet_z += f.translation().z();
  }
  com_height_ = com_offset_.z() - feet_z / static_cast<double>(legs);

  ConvexMpc::Parameters p;
  p.mass = params().get<double>("mass", rb.mass);
  p.inertia = rb.inertia;
  p.horizon = params().get<int>("horizon", p.horizon);
  p.dt = mpc_dt_;
  p.mu = params().get<double>("mu", p.mu);
  p.fz_max = params().get<double>("fz_max", p.fz_max);
  p.force_weight = params().get<double>("force_weight", p.force_weight);
  const auto w = params().get<std::vector<double>>("weights", {});
  if (w.size() == 13) {
    for (int i = 0; i < 13; ++i) {
      p.weights(i) = w[static_cast<std::size_t>(i)];
    }
  }
  mpc_.configure(p, static_cast<int>(legs));
  frozen_.assign(legs, true);
  stance_.assign(legs, true);
  liftoff_.assign(legs, Eigen::Vector3d::Zero());
  forces_.assign(legs, Eigen::Vector3d::Zero());
  logger().info() << "MPC: mass " << p.mass << " kg, CoM height " << com_height_ << " m, horizon " <<
    p.horizon << " x " << p.dt << " s";
  return true;
}

bool ConvexMpcController::stance_at(std::size_t leg, double t, const Gait & g) const
{
  if (!walking_ || frozen_[leg]) {
    return true;
  }
  const double phi = std::fmod(t / g.period + g.offsets[leg], 1.0);
  return phi < g.duty;
}

void ConvexMpcController::activate(const gaitos::RobotState & state)
{
  q_ = state.joints.position;
  hold_q_ = state.joints.position;
  cmd_.setZero();
  phase_time_ = 0.0;
  idle_ = 0.0;
  walking_ = false;
  stopping_ = false;
  std::fill(frozen_.begin(), frozen_.end(), true);
  std::fill(stance_.begin(), stance_.end(), true);
  since_solve_ = 1e9;
  if (state.base.valid) {
    p_des_ = state.base.position + state.base.orientation * com_offset_;
    yaw_des_ = gaitos::yaw_of(state.base.orientation);
  }
}

bool ConvexMpcController::update(
  const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
  gaitos::JointCommand & command)
{
  if (!state.base.valid) {
    logger().error() << "The MPC needs a valid state estimate";
    return false;
  }
  const auto legs = model().leg_count();
  const auto it = gaits_.find(reference.gait);
  const Gait & g = it != gaits_.end() ? it->second : gaits_.at(default_gait_);

  // Command
  const Eigen::Vector3d target = limited_command(reference, max_velocity_);
  for (int i = 0; i < 3; ++i) {
    cmd_(i) = gaitos::rate_limit(cmd_(i), target(i), max_acceleration_(i), dt);
  }
  const bool moving = target.norm() > 1e-3 || cmd_.norm() > 1e-3;
  if (moving) {
    idle_ = 0.0;
    stopping_ = false;
    if (!walking_) {
      walking_ = true;
      phase_time_ = 0.0;
      std::fill(frozen_.begin(), frozen_.end(), false);
    }
  } else if (walking_) {
    idle_ += dt;
    stopping_ = idle_ > idle_time_;
  }
  if (walking_) {
    phase_time_ += dt;
  }

  // State
  const Eigen::Matrix3d r = state.base.orientation.toRotationMatrix();
  const Eigen::Vector3d rpy = gaitos::quaternion_to_rpy(state.base.orientation);
  const double yaw = rpy.z();
  const Eigen::Matrix3d r_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d w_world = r * state.base.angular_velocity;
  const Eigen::Vector3d com = state.base.position + r * com_offset_;
  const Eigen::Vector3d com_vel = state.base.linear_velocity + w_world.cross(r * com_offset_);
  const Eigen::Vector3d v_cmd = r_yaw * Eigen::Vector3d(cmd_.x(), cmd_.y(), 0.0);

  // Reference, kept close to the robot
  yaw_des_ = gaitos::wrap_angle(yaw_des_ + cmd_.z() * dt);
  const double yaw_err = gaitos::wrap_angle(yaw_des_ - yaw);
  yaw_des_ = gaitos::wrap_angle(yaw + std::clamp(yaw_err, -0.2, 0.2));
  p_des_.head<2>() += v_cmd.head<2>() * dt;
  Eigen::Vector2d err = p_des_.head<2>() - com.head<2>();
  if (err.norm() > 0.1) {
    err *= 0.1 / err.norm();
  }
  p_des_.head<2>() = com.head<2>() + err;
  p_des_.z() = foot_height_ + com_height_ + reference.body_offset.z();

  // Feet
  std::vector<Eigen::Vector3d> feet_world(legs);
  for (std::size_t l = 0; l < legs; ++l) {
    feet_world[l] = state.base.position + r * model().leg(l).chain.forward(state.joints.position).translation();
  }

  // Schedule and stopping. The gait goes on, symmetric, until an instant with every foot in
  // stance both in the schedule and on the ground (measured contact), and then all the legs stop
  // together: freezing them one by one at their own touchdown leaves an uneven gait meanwhile,
  // and a foot that lands late (still in the air when its stance starts) would get forces that
  // nothing supports. Without contact sensing (or if it never comes), the schedule alone decides
  // after two more gait periods.
  bool all_down = true;
  for (std::size_t l = 0; l < legs; ++l) {
    const bool stance = stance_at(l, phase_time_, g);
    const bool touching = l >= state.contacts.size() || state.contacts.contact[l];
    all_down = all_down && stance && (touching || idle_ > idle_time_ + 2.0 * g.period);
    if (!stance && stance_[l]) {
      liftoff_[l] = feet_world[l];
    }
    stance_[l] = stance;
  }
  if (stopping_ && all_down) {
    std::fill(frozen_.begin(), frozen_.end(), true);
    walking_ = false;
    stopping_ = false;
    since_solve_ = 1e9;   // new forces now, with the four feet: the last ones were for two
    hold_q_ = state.joints.position;
  }

  // MPC at its own rate
  since_solve_ += dt;
  if (since_solve_ >= mpc_dt_ - 1e-9) {
    since_solve_ = 0.0;
    const int n = mpc_.parameters().horizon;
    Vector13 x0;
    x0 << rpy, com, w_world, com_vel, -mpc_.parameters().gravity;
    Eigen::VectorXd x_ref(13 * n);
    std::vector<std::vector<bool>> contact(static_cast<std::size_t>(n), std::vector<bool>(legs));
    for (int k = 0; k < n; ++k) {
      const double tk = (k + 1) * mpc_dt_;
      Vector13 xr;
      xr << reference.body_rpy.x(), reference.body_rpy.y(), yaw_des_ + cmd_.z() * tk,
        p_des_.x() + v_cmd.x() * tk, p_des_.y() + v_cmd.y() * tk, p_des_.z(),
        0.0, 0.0, cmd_.z(), v_cmd.x(), v_cmd.y(), 0.0, -mpc_.parameters().gravity;
      x_ref.segment<13>(13 * k) = xr;
      for (std::size_t l = 0; l < legs; ++l) {
        contact[static_cast<std::size_t>(k)][l] = stance_at(l, phase_time_ + k * mpc_dt_, g);
      }
    }
    std::vector<Eigen::Vector3d> rel(legs);
    for (std::size_t l = 0; l < legs; ++l) {
      rel[l] = feet_world[l] - com;
    }
    result_ = mpc_.solve(x0, x_ref, rel, contact, forces_);
  }

  // Joint commands
  command.position = ik_.stance_q();
  std::fill(command.velocity.begin(), command.velocity.end(), 0.0);
  std::fill(command.effort.begin(), command.effort.end(), 0.0);
  command.kp = upper_gains_.kp;
  command.kd = upper_gains_.kd;
  const double t_stance = g.duty * g.period;
  const double t_swing = (1.0 - g.duty) * g.period;
  const Eigen::Vector3d v_world_body = state.base.linear_velocity;
  for (std::size_t l = 0; l < legs; ++l) {
    const auto & chain = model().leg(l).chain;
    if (stance_[l]) {
      // Ground reaction force -> joint torques: tau = -J^T f (base frame)
      const Eigen::VectorXd tau = -chain.jacobian(state.joints.position).topRows(3).transpose() *
        (r.transpose() * forces_[l]);
      // Standing (not walking): the standing gains hold the posture of the stop on top of the
      // forces (gains.standing; by default the stance gains, i.e. torque only)
      const auto & gains = walking_ ? stance_gains_ : standing_gains_;
      for (std::size_t k = 0; k < chain.dof(); ++k) {
        const auto i = static_cast<std::size_t>(chain.joint_indices()[k]);
        command.position[i] = walking_ ? state.joints.position[i] : hold_q_[i];
        q_[i] = state.joints.position[i];
        command.effort[i] = tau(static_cast<Eigen::Index>(k));
        command.kp[i] = gains.kp[i];
        command.kd[i] = gains.kd[i];
      }
      continue;
    }
    // Swing: cycloid to the Raibert footstep, in the world
    const double phi = std::fmod(phase_time_ / g.period + g.offsets[l], 1.0);
    const double s = std::clamp((phi - g.duty) / (1.0 - g.duty), 0.0, 1.0);
    const Eigen::Vector3d nominal = ik_.nominal_feet()[l].translation();
    const Eigen::Vector3d hip(nominal.x(), nominal.y(), 0.0);
    const Eigen::Vector3d turn = Eigen::AngleAxisd(0.5 * cmd_.z() * t_stance, Eigen::Vector3d::UnitZ()) * hip - hip;
    Eigen::Vector3d td = state.base.position + r_yaw * (hip + turn) + v_world_body * (1.0 - s) * t_swing +
      0.5 * t_stance * v_cmd + raibert_gain_ * (v_world_body - v_cmd);
    td.z() = foot_height_;
    Eigen::Vector3d p = liftoff_[l] + (td - liftoff_[l]) * cycloid(s);
    p.z() = liftoff_[l].z() + (foot_height_ - liftoff_[l].z()) * s +
      0.5 * g.step_height * (1.0 - std::cos(2.0 * M_PI * s));
    Eigen::Isometry3d target = ik_.nominal_feet()[l];
    target.translation() = r.transpose() * (p - state.base.position);
    gaitos::KinematicChain::IkOptions options;
    chain.inverse(target, q_, options);
    for (std::size_t k = 0; k < chain.dof(); ++k) {
      const auto i = static_cast<std::size_t>(chain.joint_indices()[k]);
      command.position[i] = q_[i];
      command.kp[i] = swing_gains_.kp[i];
      command.kd[i] = swing_gains_.kd[i];
    }
  }
  return true;
}

}  // namespace gaitos_dynamics

PLUGINLIB_EXPORT_CLASS(gaitos_dynamics::ConvexMpcController, gaitos::ControllerBase)
