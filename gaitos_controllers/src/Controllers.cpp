//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_controllers/Controllers.hpp"

#include <algorithm>
#include <cmath>

#include "gaitos_common/Math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_controllers
{

using gaitos::Params;

// ------------------------------------------------------------------------------------------
// JointPdController

bool JointPdController::update(
  const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
  gaitos::JointCommand & command)
{
  (void)state;
  (void)dt;
  if (reference.joints.size() != command.size()) {
    return false;
  }
  command = reference.joints;
  return true;
}

// ------------------------------------------------------------------------------------------
// StanceController

bool StanceController::on_initialize()
{
  if (model().leg_count() == 0) {
    logger().error() << "The robot has no legs";
    return false;
  }
  ik_.configure(model_ptr(), params().get<std::string>("stance_pose", "stand"));
  gains_ = gaitos::read_gains(params().sub("gains"), model(), 40.0, 1.0);
  max_offset_ = params().get_vector3("max_offset", max_offset_);
  max_rpy_ = params().get_vector3("max_rpy", max_rpy_);
  max_speed_ = params().get<double>("max_speed", max_speed_);
  max_angular_speed_ = params().get<double>("max_angular_speed", max_angular_speed_);
  gravity_ = params().get<double>("gravity_compensation", gravity_);
  attitude_gain_ = params().get<double>("attitude_gain", attitude_gain_);
  blend_time_ = params().get<double>("blend_time", blend_time_);
  return true;
}

void StanceController::activate(const gaitos::RobotState & state)
{
  start_q_ = state.joints.position;
  q_ = ik_.stance_q();
  offset_.setZero();
  rpy_.setZero();
  elapsed_ = 0.0;
}

bool StanceController::update(
  const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
  gaitos::JointCommand & command)
{
  elapsed_ += dt;
  for (int i = 0; i < 3; ++i) {
    const double o = std::clamp(reference.body_offset(i), -max_offset_(i), max_offset_(i));
    const double a = std::clamp(reference.body_rpy(i), -max_rpy_(i), max_rpy_(i));
    offset_(i) = gaitos::rate_limit(offset_(i), o, max_speed_, dt);
    rpy_(i) = gaitos::rate_limit(rpy_(i), a, max_angular_speed_, dt);
  }
  Eigen::Vector3d rpy = rpy_;
  if (attitude_gain_ > 0.0 && state.base.valid) {
    const Eigen::Vector3d measured = gaitos::quaternion_to_rpy(state.base.orientation);
    rpy.x() += attitude_gain_ * (rpy_.x() - measured.x());
    rpy.y() += attitude_gain_ * (rpy_.y() - measured.y());
  }
  ik_.solve(ik_.body_pose_feet(ik_.nominal_feet(), offset_, rpy), q_);

  const double alpha = blend_time_ > 0.0 ? std::clamp(elapsed_ / blend_time_, 0.0, 1.0) : 1.0;
  blend(start_q_, q_, alpha, command.position);
  std::fill(command.velocity.begin(), command.velocity.end(), 0.0);
  std::fill(command.effort.begin(), command.effort.end(), 0.0);
  command.kp = gains_.kp;
  command.kd = gains_.kd;
  if (gravity_ > 0.0) {
    const std::vector<bool> all(model().leg_count(), true);
    ik_.gravity_compensation(
      command.position, state.imu.orientation, all, gravity_ * alpha, command.effort);
  }
  return true;
}

// ------------------------------------------------------------------------------------------
// GaitController

bool GaitController::on_initialize()
{
  const auto legs = model().leg_count();
  if (legs == 0) {
    logger().error() << "The robot has no legs";
    return false;
  }
  ik_.configure(model_ptr(), params().get<std::string>("stance_pose", "stand"));
  const Params gaits = params().sub("gaits");
  for (const auto & name : gaits.keys()) {
    const Params g = gaits.sub(name);
    GaitPattern p;
    p.period = g.get<double>("period", p.period);
    p.duty = g.get<double>("duty", p.duty);
    p.step_height = g.get<double>("step_height", p.step_height);
    p.sway = g.get<double>("sway", p.sway);
    p.offsets = g.get<std::vector<double>>("offsets", {});
    if (p.offsets.size() != legs || p.period <= 0.0 || p.duty <= 0.0 || p.duty >= 1.0) {
      logger().error() << "Gait " << name << ": needs " << legs <<
        " offsets, period > 0 and 0 < duty < 1";
      return false;
    }
    gaits_[name] = p;
  }
  if (gaits_.empty()) {
    // Default: legs alternate in two groups (trot for 4 legs, walk for 2)
    GaitPattern p;
    for (std::size_t l = 0; l < legs; ++l) {
      p.offsets.push_back((legs == 4) ? ((l == 0 || l == 3) ? 0.0 : 0.5) :
        static_cast<double>(l) / static_cast<double>(legs));
    }
    gaits_["trot"] = p;
  }
  default_gait_ = params().get<std::string>("default_gait", gaits_.begin()->first);
  if (!gaits_.count(default_gait_)) {
    logger().error() << "Unknown default gait " << default_gait_;
    return false;
  }
  max_velocity_ = params().get_vector3("max_velocity", max_velocity_);
  max_acceleration_ = params().get_vector3("max_acceleration", max_acceleration_);
  raibert_gain_ = params().get<double>("raibert_gain", raibert_gain_);
  feedback_kp_ = params().get_vector3("velocity_feedback.kp", feedback_kp_);
  feedback_ki_ = params().get_vector3("velocity_feedback.ki", feedback_ki_);
  feedback_max_ = params().get_vector3("velocity_feedback.max_integral", feedback_max_);
  gravity_ = params().get<double>("gravity_compensation", gravity_);
  idle_time_ = params().get<double>("idle_time", idle_time_);
  blend_time_ = params().get<double>("blend_time", blend_time_);
  max_sway_speed_ = params().get<double>("max_sway_speed", max_sway_speed_);
  attitude_gain_ = params().get<double>("attitude_gain", attitude_gain_);
  stance_gains_ = gaitos::read_gains(params().sub("gains.stance"), model(), 40.0, 1.0);
  swing_gains_ = gaitos::read_gains(params().sub("gains.swing"), model(), 30.0, 1.0);
  upper_gains_ = gaitos::read_gains(params().sub("gains.upper_body"), model(), 40.0, 1.0);
  stance_.assign(legs, true);
  frozen_.assign(legs, true);
  foot_.resize(legs);
  liftoff_.resize(legs);
  foot_yaw_.assign(legs, 0.0);
  liftoff_yaw_.assign(legs, 0.0);
  return true;
}

const GaitPattern & GaitController::pattern(const std::string & name) const
{
  const auto it = gaits_.find(name);
  return it != gaits_.end() ? it->second : gaits_.at(default_gait_);
}

void GaitController::activate(const gaitos::RobotState & state)
{
  start_q_ = state.joints.position;
  q_ = ik_.stance_q();
  elapsed_ = 0.0;
  cmd_.setZero();
  integral_.setZero();
  sway_.setZero();
  phase_time_ = 0.0;
  idle_ = 0.0;
  walking_ = false;
  stopping_ = false;
  std::fill(stance_.begin(), stance_.end(), true);
  std::fill(frozen_.begin(), frozen_.end(), true);
  for (std::size_t l = 0; l < foot_.size(); ++l) {
    foot_[l] = ik_.nominal_feet()[l].translation();
  }
  std::fill(foot_yaw_.begin(), foot_yaw_.end(), 0.0);
}

bool GaitController::update(
  const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
  gaitos::JointCommand & command)
{
  elapsed_ += dt;
  const auto & g = pattern(reference.gait);
  const auto legs = model().leg_count();

  // Command, limited in value and rate
  const Eigen::Vector3d target(
    std::clamp(reference.linear_velocity.x(), -max_velocity_.x(), max_velocity_.x()),
    std::clamp(reference.linear_velocity.y(), -max_velocity_.y(), max_velocity_.y()),
    std::clamp(reference.angular_velocity.z(), -max_velocity_.z(), max_velocity_.z()));
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

  // Velocity loop with the estimated body velocity
  Eigen::Vector3d planned = cmd_;
  if (walking_ && !stopping_ && state.base.valid) {
    const Eigen::Vector3d vb = state.base.linear_velocity_body();
    const Eigen::Vector3d error = cmd_ - Eigen::Vector3d(vb.x(), vb.y(), state.base.angular_velocity.z());
    for (int i = 0; i < 3; ++i) {
      integral_(i) = std::clamp(integral_(i) + feedback_ki_(i) * error(i) * dt,
          -feedback_max_(i), feedback_max_(i));
    }
    planned += feedback_kp_.cwiseProduct(error) + integral_;
  } else {
    integral_ *= std::max(0.0, 1.0 - 5.0 * dt);   // fades out when standing
  }
  const Eigen::Vector3d v(planned.x(), planned.y(), 0.0);
  const Eigen::Vector3d w(0.0, 0.0, planned.z());
  const Eigen::Vector3d v_meas = state.base.valid ? state.base.linear_velocity_body() : v;
  const double t_stance = g.duty * g.period;
  const double ground = -ik_.nominal_height();
  std::vector<Eigen::Vector3d> previous = foot_;

  for (std::size_t l = 0; l < legs; ++l) {
    double phi = 0.0;
    bool stance = true;
    if (!frozen_[l]) {
      phi = std::fmod(phase_time_ / g.period + g.offsets[l], 1.0);
      stance = phi < g.duty;
    }
    if (stopping_ && stance) {
      frozen_[l] = true;
    }
    if (!stance && stance_[l]) {
      liftoff_[l] = foot_[l];
      liftoff_yaw_[l] = foot_yaw_[l];
    }
    stance_[l] = stance;
    if (stance) {
      Eigen::Vector3d dp = -(v + w.cross(foot_[l])) * dt;
      foot_[l].x() += dp.x();
      foot_[l].y() += dp.y();
      foot_[l].z() = ground;
      foot_yaw_[l] -= w.z() * dt;   // a planted foot turns against the body
    } else {
      const double s = (phi - g.duty) / (1.0 - g.duty);
      const Eigen::Vector3d nominal = ik_.nominal_feet()[l].translation();
      Eigen::Vector3d td = Eigen::AngleAxisd(0.5 * w.z() * t_stance, Eigen::Vector3d::UnitZ()) * nominal +
        0.5 * t_stance * v + raibert_gain_ * (v_meas - v);
      const double c = s - std::sin(2.0 * M_PI * s) / (2.0 * M_PI);
      foot_[l].x() = liftoff_[l].x() + (td.x() - liftoff_[l].x()) * c;
      foot_[l].y() = liftoff_[l].y() + (td.y() - liftoff_[l].y()) * c;
      foot_[l].z() = ground + 0.5 * g.step_height * (1.0 - std::cos(2.0 * M_PI * s));
      foot_yaw_[l] = liftoff_yaw_[l] + (0.5 * w.z() * t_stance - liftoff_yaw_[l]) * c;
    }
  }
  if (stopping_ && std::all_of(frozen_.begin(), frozen_.end(), [](bool f) {return f;})) {
    walking_ = false;
    stopping_ = false;
  }

  // Lateral sway: the body moves over the stance feet, away from the swinging ones
  Eigen::Vector2d sway_target = Eigen::Vector2d::Zero();
  if (g.sway > 0.0 && walking_ && !stopping_) {
    const double mid = 0.5 * (1.0 + g.duty);
    for (std::size_t l = 0; l < legs; ++l) {
      const Eigen::Vector2d side = ik_.nominal_feet()[l].translation().head<2>().normalized();
      const double phi = std::fmod(phase_time_ / g.period + g.offsets[l], 1.0);
      sway_target -= g.sway * 0.5 * (1.0 + std::cos(2.0 * M_PI * (phi - mid))) * side;
    }
    // Only the lateral component
    sway_target.x() = 0.0;
  }
  for (int i = 0; i < 2; ++i) {
    sway_(i) = gaitos::rate_limit(sway_(i), sway_target(i), max_sway_speed_, dt);
  }

  // Feet to joints
  std::vector<Eigen::Isometry3d> feet(legs);
  for (std::size_t l = 0; l < legs; ++l) {
    feet[l] = ik_.nominal_feet()[l];
    feet[l].linear() = Eigen::AngleAxisd(foot_yaw_[l], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      feet[l].linear();
    feet[l].translation() = foot_[l];
    feet[l].translation().head<2>() -= sway_;
  }
  Eigen::Vector3d rpy = reference.body_rpy;
  if (attitude_gain_ > 0.0 && state.base.valid) {
    const Eigen::Vector3d measured = gaitos::quaternion_to_rpy(state.base.orientation);
    rpy.x() += attitude_gain_ * (reference.body_rpy.x() - measured.x());
    rpy.y() += attitude_gain_ * (reference.body_rpy.y() - measured.y());
  }
  const auto targets = ik_.body_pose_feet(feet, reference.body_offset, rpy);
  ik_.solve(targets, q_);

  const double alpha = blend_time_ > 0.0 ? std::clamp(elapsed_ / blend_time_, 0.0, 1.0) : 1.0;
  blend(start_q_, q_, alpha, command.position);
  std::fill(command.velocity.begin(), command.velocity.end(), 0.0);
  std::fill(command.effort.begin(), command.effort.end(), 0.0);
  command.kp = upper_gains_.kp;
  command.kd = upper_gains_.kd;
  for (std::size_t l = 0; l < legs; ++l) {
    if (dt > 0.0 && alpha >= 1.0) {
      const Eigen::Vector3d foot_velocity = gaitos::rpy_to_matrix(reference.body_rpy).transpose() *
        (foot_[l] - previous[l]) / dt;
      ik_.leg_velocity(l, q_, foot_velocity, command.velocity);
    }
    const auto & gains = stance_[l] ? stance_gains_ : swing_gains_;
    for (const int idx : model().leg(l).chain.joint_indices()) {
      const auto i = static_cast<std::size_t>(idx);
      command.kp[i] = gains.kp[i];
      command.kd[i] = gains.kd[i];
    }
  }
  if (gravity_ > 0.0) {
    ik_.gravity_compensation(command.position, state.imu.orientation, stance_, gravity_ * alpha,
      command.effort);
  }
  return true;
}

}  // namespace gaitos_controllers

PLUGINLIB_EXPORT_CLASS(gaitos_controllers::JointPdController, gaitos::ControllerBase)
PLUGINLIB_EXPORT_CLASS(gaitos_controllers::StanceController, gaitos::ControllerBase)
PLUGINLIB_EXPORT_CLASS(gaitos_controllers::GaitController, gaitos::ControllerBase)
