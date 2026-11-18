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
// LipmZmpController

bool LipmZmpController::on_initialize()
{
  if (model().leg_count() != 2) {
    logger().error() << "LipmZmpController is for bipeds (2 legs), the robot has " << model().leg_count();
    return false;
  }
  const auto pose = params().get<std::string>("stance_pose", "stand");
  ik_.configure(model_ptr(), pose);
  const auto & nominal = ik_.nominal_feet();
  side_ = nominal[0].translation().y() >= nominal[1].translation().y() ? std::array<int, 2>{{0, 1}} :
    std::array<int, 2>{{1, 0}};
  const double width = std::abs(nominal[0].translation().y() - nominal[1].translation().y());
  step_width_ = params().get<double>("step_width", width);
  step_time_ = params().get<double>("step_time", step_time_);
  double_support_ = std::clamp(params().get<double>("double_support", double_support_), 0.0, 0.9);
  step_height_ = params().get<double>("step_height", step_height_);
  start_delay_ = params().get<double>("start_delay", start_delay_);
  idle_time_ = params().get<double>("idle_time", idle_time_);
  attitude_gain_ = params().get<double>("attitude_gain", attitude_gain_);
  com_fb_p_ = params().get<double>("com_feedback.position", com_fb_p_);
  com_fb_v_ = params().get<double>("com_feedback.velocity", com_fb_v_);
  dt_ = params().get<double>("dt", dt_);
  const Eigen::Vector3d center = params().get_vector3("foot_center", Eigen::Vector3d::Zero());
  foot_center_ = center.head<2>();
  sole_depth_ = center.z();
  feedforward_ = std::clamp(params().get<double>("feedforward", feedforward_), 0.0, 1.0);
  heading_gain_ = params().get<double>("heading_gain", heading_gain_);
  heading_body_gain_ = params().get<double>("heading_body_gain", heading_body_gain_);
  dcm_gain_ = params().get<double>("dcm_feedback", dcm_gain_);
  dcm_position_limit_ = params().get<double>("dcm_position_limit", dcm_position_limit_);
  const Eigen::Vector3d half = params().get_vector3("sole_half_size", Eigen::Vector3d(0.08, 0.03, 0.0));
  sole_half_ = half.head<2>();
  feedback_ki_ = params().get_vector3("velocity_feedback.ki", feedback_ki_);
  feedback_max_ = params().get_vector3("velocity_feedback.max_integral", feedback_max_);
  max_velocity_ = params().get_vector3("max_velocity", max_velocity_);
  max_acceleration_ = params().get_vector3("max_acceleration", max_acceleration_);
  gains_ = gaitos::read_gains(params().sub("gains"), model(), 100.0, 2.0);

  const auto rb = model().tree().rigid_body(ik_.stance_q(), model().base_link());
  com_offset_ = rb.com;
  mass_ = params().get<double>("mass", rb.mass);
  ground_z_ = 0.5 * (nominal[0].translation().z() + nominal[1].translation().z());
  const double zc = com_offset_.z() - ground_z_;
  preview_.configure(dt_, zc, params().get<double>("preview_time", 1.6),
    params().get<double>("zmp_weight", 1.0), params().get<double>("jerk_weight", 1e-6));
  foot_rotation_ = {nominal[0].linear(), nominal[1].linear()};
  logger().info() << "LIPM: CoM height " << zc << " m, step width " << step_width_ << " m, " <<
    preview_.preview_steps() << " preview samples";
  return step_time_ > 0.0;
}

Eigen::Vector2d LipmZmpController::sole(const Foot & f) const
{
  return f.p + rot2(f.yaw) * foot_center_;
}

Eigen::Vector2d LipmZmpController::middle() const
{
  return 0.5 * (sole(feet_[0]) + sole(feet_[1]));
}


LipmZmpController::Foot LipmZmpController::place(const Foot & path, int leg) const
{
  const double sign = leg == side_[0] ? 1.0 : -1.0;
  Foot f;
  f.yaw = path.yaw;
  f.p = path.p + rot2(path.yaw) * Eigen::Vector2d(0.0, sign * 0.5 * step_width_);
  return f;
}

void LipmZmpController::activate(const gaitos::RobotState & state)
{
  start_q_ = state.joints.position;
  q_ = ik_.stance_q();
  // Plan frame: the base at activation, levelled
  feet_.assign(2, Foot());
  // Nominal stance (not the measured one: under load the joints have yielded, and taking
  // that pose as the target would remove the error that holds the robot)
  for (std::size_t l = 0; l < 2; ++l) {
    const auto & f = ik_.nominal_feet()[l];
    feet_[l].p = f.translation().head<2>();
    feet_[l].yaw = gaitos::yaw_of(Eigen::Quaterniond(f.linear() * foot_rotation_[l].transpose()));
  }
  // Standing: the ZMP stays under the CoM it starts with (no transient on activation)
  preview_.reset(com_offset_.head<2>());
  stand_point_ = com_offset_.head<2>();
  plan_yaw_ = 0.0;
  t_ = 0.0;
  walking_ = false;
  stopping_ = false;
  idle_ = 0.0;
  cmd_.setZero();
  integral_.setZero();
  steps_.clear();
  zmp_buffer_.assign(static_cast<std::size_t>(preview_.preview_steps() + 1), stand_point_);
  odom_from_plan_.setIdentity();
  if (state.base.valid) {
    odom_from_plan_.translation() = state.base.position;
    odom_from_plan_.linear() = Eigen::AngleAxisd(gaitos::yaw_of(state.base.orientation),
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }
}

void LipmZmpController::replan(const Eigen::Vector3d & cmd)
{
  const double t_step = step_time_;
  auto advance = [&](const Foot & path) {
      Foot out;
      out.yaw = gaitos::wrap_angle(path.yaw + cmd.z() * t_step);
      out.p = path.p + rot2(path.yaw + 0.5 * cmd.z() * t_step) *
        Eigen::Vector2d(cmd.x() * t_step, cmd.y() * t_step);
      return out;
    };
  const bool zero = cmd.norm() < 1e-6;
  if (steps_.empty()) {
    Step s;
    s.t0 = t_ + start_delay_;
    // Step first with the foot on the side of the lateral motion
    s.support = cmd.y() > 1e-6 ? side_[1] : side_[0];
    const int swing = 1 - s.support;
    s.support_foot = feet_[static_cast<std::size_t>(s.support)];
    s.swing_from = feet_[static_cast<std::size_t>(swing)];
    Foot path;
    path.p = middle();
    path.yaw = mean_angle(feet_[0].yaw, feet_[1].yaw);
    s.swing_to = place(advance(path), swing);
    s.zmp_from = stand_point_;
    s.zero = zero;
    steps_.push_back(s);
  }
  // The current step is kept; the rest follow the command
  while (steps_.size() > 1) {
    steps_.pop_back();
  }
  const double horizon = t_ + dt_ * preview_.preview_steps() + step_time_;
  while (steps_.back().support >= 0 && steps_.back().t0 < horizon) {
    const Step & prev = steps_.back();
    Step s;
    s.t0 = prev.t0 + t_step;
    s.zmp_from = sole(prev.support_foot);
    if (stopping_ && prev.zero) {
      s.support = -1;   // final: ZMP to the middle of the feet, then stand
      s.support_foot.p = 0.5 * (sole(prev.support_foot) + sole(prev.swing_to));
      s.support_foot.yaw = mean_angle(prev.support_foot.yaw, prev.swing_to.yaw);
      steps_.push_back(s);
      break;
    }
    s.support = 1 - prev.support;
    s.support_foot = prev.swing_to;
    s.swing_from = prev.support_foot;
    Foot path;
    path.p = 0.5 * (prev.support_foot.p + prev.swing_to.p);
    path.yaw = prev.swing_to.yaw;
    s.swing_to = place(stopping_ ? path : advance(path), prev.support);
    s.zero = zero || stopping_;
    steps_.push_back(s);
  }
}

Eigen::Vector2d LipmZmpController::zmp_at(double t) const
{
  if (!walking_ || steps_.empty() || t < steps_.front().t0) {
    return stand_point_;
  }
  const double t_ds = double_support_ * step_time_;
  // Final steps store the middle of the soles; the others their support foot
  auto target = [this](const Step & s) {return s.support < 0 ? s.support_foot.p : sole(s.support_foot);};
  for (const auto & s : steps_) {
    const double duration = s.support < 0 ? t_ds : step_time_;
    if (t < s.t0 + duration || &s == &steps_.back()) {
      const double tau = t - s.t0;
      if (tau < t_ds && t_ds > 0.0) {
        return s.zmp_from + (target(s) - s.zmp_from) * gaitos::smoothstep(tau / t_ds);
      }
      return target(s);
    }
  }
  return target(steps_.back());
}

bool LipmZmpController::update(
  const gaitos::RobotState & state, const gaitos::ControlReference & reference, double dt,
  gaitos::JointCommand & command)
{
  t_ += dt;
  const Eigen::Vector3d target = limited_command(reference, max_velocity_);
  for (int i = 0; i < 3; ++i) {
    cmd_(i) = gaitos::rate_limit(cmd_(i), target(i), max_acceleration_(i), dt);
  }
  const bool moving = target.norm() > 1e-3;
  if (moving) {
    idle_ = 0.0;
    stopping_ = false;
    if (!walking_) {
      walking_ = true;
      steps_.clear();
    }
  } else if (walking_) {
    idle_ += dt;
    stopping_ = idle_ > idle_time_;
  }

  // Finished steps
  while (walking_ && !steps_.empty()) {
    const Step & s = steps_.front();
    const double duration = s.support < 0 ? double_support_ * step_time_ : step_time_;
    if (t_ < s.t0 + duration) {
      break;
    }
    if (s.support < 0) {
      walking_ = false;
      stopping_ = false;
      stand_point_ = s.support_foot.p;   // middle of the soles
      steps_.clear();
      break;
    }
    feet_[static_cast<std::size_t>(1 - s.support)] = s.swing_to;
    steps_.pop_front();
  }
  if (walking_) {
    Eigen::Vector3d planned = cmd_;
    if (state.base.valid && !stopping_) {
      // Heading: the measured body yaw (plan frame) follows the planned one
      const double yaw_meas = gaitos::wrap_angle(gaitos::yaw_of(state.base.orientation) -
          gaitos::yaw_of(Eigen::Quaterniond(odom_from_plan_.linear())));
      const double yaw_plan = plan_yaw_;   // body yaw planned (swing foot included)
      heading_error_ = gaitos::wrap_angle(yaw_plan - yaw_meas);
      planned.z() += heading_gain_ * heading_error_;
      // Body velocity: PI on the estimate
      const Eigen::Vector3d vb = state.base.linear_velocity_body();
      const Eigen::Vector3d error = cmd_ - Eigen::Vector3d(vb.x(), vb.y(), state.base.angular_velocity.z());
      for (int i = 0; i < 3; ++i) {
        integral_(i) = std::clamp(integral_(i) + feedback_ki_(i) * error(i) * dt,
            -feedback_max_(i), feedback_max_(i));
      }
      planned += integral_;
    } else {
      integral_.setZero();
      heading_error_ = 0.0;
    }
    for (int i = 0; i < 3; ++i) {
      planned(i) = std::clamp(planned(i), -max_velocity_(i), max_velocity_(i));
    }
    replan(stopping_ ? Eigen::Vector3d::Zero() : planned);
  }

  // Planned feet now (frame of the plan)
  std::vector<Foot> feet = feet_;
  std::vector<double> lift(2, 0.0);
  if (walking_ && !steps_.empty() && steps_.front().support >= 0 && t_ >= steps_.front().t0) {
    const Step & s = steps_.front();
    const double t_ds = double_support_ * step_time_;
    const double tau = t_ - s.t0;
    if (tau > t_ds) {
      const double u = (tau - t_ds) / (step_time_ - t_ds);
      const auto swing = static_cast<std::size_t>(1 - s.support);
      feet[swing].p = s.swing_from.p + (s.swing_to.p - s.swing_from.p) * cycloid(u);
      feet[swing].yaw = s.swing_from.yaw + gaitos::wrap_angle(s.swing_to.yaw - s.swing_from.yaw) * gaitos::smoothstep(u);
      lift[swing] = 0.5 * step_height_ * (1.0 - std::cos(2.0 * M_PI * u));
    }
  }

  // CoM from the preview controller
  const int n = preview_.preview_steps();
  zmp_buffer_.resize(static_cast<std::size_t>(n + 1));
  for (int j = 0; j <= n; ++j) {
    zmp_buffer_[static_cast<std::size_t>(j)] = zmp_at(t_ + j * dt_);
  }
  zmp_ref_now_ = zmp_buffer_.front();
  const int substeps = std::max(1, static_cast<int>(std::lround(dt / dt_)));
  for (int k = 0; k < substeps; ++k) {
    preview_.step(zmp_buffer_);
  }
  if ((com_fb_p_ > 0.0 || com_fb_v_ > 0.0) && state.base.valid) {
    const Eigen::Isometry3d plan_from_odom = odom_from_plan_.inverse();
    const Eigen::Vector3d com = plan_from_odom * (state.base.position + state.base.orientation * com_offset_);
    const Eigen::Vector3d vel = plan_from_odom.linear() * state.base.linear_velocity;
    preview_.correct(com.head<2>(), vel.head<2>(), com_fb_p_, com_fb_v_);
  }

  // Base pose in the plan frame
  const double yaw = mean_angle(feet[0].yaw, feet[1].yaw);
  plan_yaw_ = yaw;
  Eigen::Vector3d rpy(reference.body_rpy.x(), reference.body_rpy.y(), 0.0);
  if (attitude_gain_ > 0.0 && state.base.valid) {
    const Eigen::Vector3d measured = gaitos::quaternion_to_rpy(state.base.orientation);
    rpy.x() += attitude_gain_ * (reference.body_rpy.x() - measured.x());
    rpy.y() += attitude_gain_ * (reference.body_rpy.y() - measured.y());
  }
  Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
  // Heading recovered over the stance foot: the body turns towards the planned yaw
  const double body_yaw = yaw + std::clamp(heading_body_gain_ * heading_error_, -0.2, 0.2);
  base.linear() = Eigen::AngleAxisd(body_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    gaitos::rpy_to_matrix(rpy);
  const Eigen::Vector2d com = preview_.com();
  const Eigen::Vector3d offset_world = base.linear() * com_offset_;
  base.translation() = Eigen::Vector3d(com.x() - offset_world.x(), com.y() - offset_world.y(),
      reference.body_offset.z());

  // Feet in the base frame and inverse kinematics
  std::vector<Eigen::Isometry3d> targets(2);
  const Eigen::Isometry3d base_inv = base.inverse();
  for (std::size_t l = 0; l < 2; ++l) {
    Eigen::Isometry3d f = Eigen::Isometry3d::Identity();
    f.linear() = Eigen::AngleAxisd(feet[l].yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * foot_rotation_[l];
    f.translation() = Eigen::Vector3d(feet[l].p.x(), feet[l].p.y(), ground_z_ + lift[l]);
    targets[l] = base_inv * f;
  }
  ik_.solve(targets, q_);

  const double alpha = std::clamp(t_ / 0.3, 0.0, 1.0);
  gaitos_controllers::blend(start_q_, q_, alpha, command.position);
  std::fill(command.velocity.begin(), command.velocity.end(), 0.0);
  std::fill(command.effort.begin(), command.effort.end(), 0.0);
  command.kp = gains_.kp;
  command.kd = gains_.kd;

  // Feedforward: ground reaction of the LIPM at the center of pressure of each stance foot
  if (feedforward_ > 0.0) {
    const Eigen::Vector2d acc = preview_.com_acceleration();
    const Eigen::Vector3d force(mass_ * acc.x(), mass_ * acc.y(), mass_ * 9.81);
    Eigen::Vector2d zmp = preview_.zmp();
    // DCM stabilizer: move the center of pressure with the error of the measured DCM
    if (dcm_gain_ > 0.0 && state.base.valid) {
      const double omega = std::sqrt(9.81 / preview_.com_height());
      const Eigen::Isometry3d plan_from_odom = odom_from_plan_.inverse();
      const Eigen::Vector3d c = plan_from_odom * (state.base.position + state.base.orientation * com_offset_);
      const Eigen::Vector3d w = state.base.orientation * state.base.angular_velocity;
      const Eigen::Vector3d cv = plan_from_odom.linear() *
        (state.base.linear_velocity + w.cross(state.base.orientation * com_offset_));
      const Eigen::Vector2d position_error =
        (c.head<2>() - preview_.com()).cwiseMax(-dcm_position_limit_).cwiseMin(dcm_position_limit_);
      const Eigen::Vector2d velocity_error = cv.head<2>() - preview_.com_velocity();
      zmp += dcm_gain_ * (position_error + velocity_error / omega);
    }
    auto clamp_to_sole = [this](const Foot & f, const Eigen::Vector2d & p) {
        const Eigen::Matrix2d r = rot2(f.yaw);
        Eigen::Vector2d local = r.transpose() * (p - sole(f));
        local = local.cwiseMax(-sole_half_).cwiseMin(sole_half_);
        return Eigen::Vector2d(sole(f) + r * local);
      };
    std::array<double, 2> share{{0.0, 0.0}};
    std::array<Eigen::Vector2d, 2> cop{{sole(feet[0]), sole(feet[1])}};
    if (lift[0] > 1e-4 || lift[1] > 1e-4) {
      const std::size_t stance = lift[0] > 1e-4 ? 1 : 0;   // single support
      share[stance] = 1.0;
      cop[stance] = clamp_to_sole(feet[stance], zmp);
    } else {
      const Eigen::Vector2d d = cop[1] - cop[0];
      const double t = d.squaredNorm() > 1e-9 ? std::clamp((zmp - cop[0]).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.5;
      share = {{1.0 - t, t}};
    }
    for (std::size_t l = 0; l < 2; ++l) {
      if (share[l] <= 0.0) {
        continue;
      }
      const Eigen::Vector3d f = share[l] * force;
      const Eigen::Vector3d ankle(feet[l].p.x(), feet[l].p.y(), ground_z_);
      const Eigen::Vector3d p_cop(cop[l].x(), cop[l].y(), ground_z_ + sole_depth_);
      const Eigen::Vector3d m = (p_cop - ankle).cross(f);
      Eigen::Matrix<double, 6, 1> wrench;
      wrench << base.linear().transpose() * f, base.linear().transpose() * m;
      const auto & chain = model().leg(l).chain;
      const Eigen::MatrixXd j = chain.jacobian(command.position);
      const Eigen::VectorXd tau = -j.transpose() * wrench;
      for (std::size_t k = 0; k < chain.dof(); ++k) {
        command.effort[static_cast<std::size_t>(chain.joint_indices()[k])] +=
          feedforward_ * alpha * tau(static_cast<Eigen::Index>(k));
      }
    }
  }
  return true;
}

}  // namespace gaitos_dynamics


PLUGINLIB_EXPORT_CLASS(gaitos_dynamics::LipmZmpController, gaitos::ControllerBase)
