//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_controllers/LegIk.hpp"

#include <algorithm>

#include "gaitos_common/Math.hpp"

namespace gaitos_controllers
{

void LegIk::configure(gaitos::RobotModelPtr model, const std::string & stance_pose)
{
  model_ = std::move(model);
  stance_q_ = model_->pose(stance_pose);
  nominal_.clear();
  height_ = 0.0;
  for (const auto & leg : model_->legs()) {
    nominal_.push_back(leg.chain.forward(stance_q_));
    height_ -= nominal_.back().translation().z();
  }
  if (!nominal_.empty()) {
    height_ /= static_cast<double>(nominal_.size());
  }
  // Legs sharing a joint (a mimic joint drives both: NAO HipYawPitch) are solved together
  coupled_ = false;
  std::vector<int> seen;
  for (const auto & leg : model_->legs()) {
    for (const int i : leg.chain.joint_indices()) {
      coupled_ = coupled_ || std::find(seen.begin(), seen.end(), i) != seen.end();
    }
    seen.insert(seen.end(), leg.chain.joint_indices().begin(), leg.chain.joint_indices().end());
  }
}

namespace
{
/// Rotation vector taking `current` to `target`
Eigen::Vector3d rotation_error(const Eigen::Matrix3d & target, const Eigen::Matrix3d & current)
{
  const Eigen::AngleAxisd aa(target * current.transpose());
  return aa.angle() * aa.axis();
}
}  // namespace

double LegIk::solve(const std::vector<Eigen::Isometry3d> & feet, std::vector<double> & q) const
{
  double error = 0.0;
  if (coupled_) {
    // Every foot at once: damped least squares on the stacked jacobians, over the union of
    // the joints of the legs (the shared joint takes the value that suits all the feet).
    // The yaw common to every foot is left free: with a shared yaw joint (NAO HipYawPitch, on
    // mirrored axes) the feet only turn against each other relative to the body, so only
    // their relative yaw is reachable; the body turns over the stance foot instead.
    const std::size_t legs = std::min(model_->leg_count(), feet.size());
    std::vector<int> joints;
    for (std::size_t l = 0; l < legs; ++l) {
      for (const int i : model_->leg(l).chain.joint_indices()) {
        if (std::find(joints.begin(), joints.end(), i) == joints.end()) {
          joints.push_back(i);
        }
      }
    }
    gaitos::KinematicChain::IkOptions options;
    Eigen::Index rows = 0;
    for (std::size_t l = 0; l < legs; ++l) {
      rows += model_->leg(l).orientation_ik ? 6 : 3;
    }
    const auto n = static_cast<Eigen::Index>(joints.size());
    for (int it = 0; it < 2 * options.max_iterations; ++it) {
      Eigen::MatrixXd j = Eigen::MatrixXd::Zero(rows, n);
      Eigen::VectorXd e(rows);
      Eigen::Index r = 0;
      error = 0.0;
      std::vector<Eigen::Vector3d> rot(legs, Eigen::Vector3d::Zero());
      double common_yaw = 0.0;
      int oriented = 0;
      for (std::size_t l = 0; l < legs; ++l) {
        if (model_->leg(l).orientation_ik) {
          const Eigen::Isometry3d tip = model_->leg(l).chain.forward(q);
          rot[l] = rotation_error(feet[l].linear(), tip.linear());
          common_yaw += rot[l].z();
          ++oriented;
        }
      }
      common_yaw = oriented > 1 ? common_yaw / oriented : 0.0;
      for (std::size_t l = 0; l < legs; ++l) {
        const auto & leg = model_->leg(l);
        Eigen::Isometry3d tip;
        const Eigen::MatrixXd jac = leg.chain.jacobian(q, &tip);
        const int m = leg.orientation_ik ? 6 : 3;
        e.segment<3>(r) = feet[l].translation() - tip.translation();
        error = std::max(error, e.segment<3>(r).norm());
        if (leg.orientation_ik) {
          rot[l].z() -= common_yaw;
          e.segment<3>(r + 3) = options.orientation_weight * rot[l];
          error = std::max(error, e.segment<3>(r + 3).norm());
        }
        for (std::size_t k = 0; k < leg.chain.dof(); ++k) {
          const auto col = std::find(joints.begin(), joints.end(), leg.chain.joint_indices()[k]) - joints.begin();
          j.block(r, col, 3, 1) += jac.block(0, static_cast<Eigen::Index>(k), 3, 1);
          if (leg.orientation_ik) {
            j.block(r + 3, col, 3, 1) += options.orientation_weight * jac.block(3, static_cast<Eigen::Index>(k), 3, 1);
          }
        }
        r += m;
      }
      if (error < options.tolerance) {
        break;
      }
      const Eigen::MatrixXd jjt = j * j.transpose() + options.damping * Eigen::MatrixXd::Identity(rows, rows);
      Eigen::VectorXd dq = j.transpose() * jjt.ldlt().solve(e);
      const double max_abs = dq.cwiseAbs().maxCoeff();
      if (max_abs > options.max_step) {
        dq *= options.max_step / max_abs;
      }
      for (Eigen::Index k = 0; k < n; ++k) {
        const auto i = static_cast<std::size_t>(joints[static_cast<std::size_t>(k)]);
        const auto & info = model_->joint(i);
        q[i] = std::clamp(q[i] + dq(k), info.lower, info.upper);
      }
    }
    return error;
  }
  for (std::size_t l = 0; l < model_->leg_count() && l < feet.size(); ++l) {
    const auto & leg = model_->leg(l);
    gaitos::KinematicChain::IkOptions options;
    options.orientation = leg.orientation_ik;
    error = std::max(error, leg.chain.inverse(feet[l], q, options));
  }
  return error;
}

void LegIk::leg_velocity(
  std::size_t leg, const std::vector<double> & q, const Eigen::Vector3d & foot_velocity,
  std::vector<double> & dq) const
{
  const auto & chain = model_->leg(leg).chain;
  const Eigen::MatrixXd j = chain.jacobian(q).topRows(3);
  const Eigen::Matrix3d jjt = j * j.transpose() + 1e-4 * Eigen::Matrix3d::Identity();
  const Eigen::VectorXd v = j.transpose() * jjt.ldlt().solve(foot_velocity);
  for (std::size_t k = 0; k < chain.dof(); ++k) {
    dq[static_cast<std::size_t>(chain.joint_indices()[k])] = v(static_cast<Eigen::Index>(k));
  }
}

void LegIk::gravity_compensation(
  const std::vector<double> & q, const Eigen::Quaterniond & orientation,
  const std::vector<bool> & stance, double fraction, std::vector<double> & tau) const
{
  const auto n = static_cast<double>(std::count(stance.begin(), stance.end(), true));
  if (n < 1.0 || fraction <= 0.0) {
    return;
  }
  const Eigen::Vector3d f_world(0.0, 0.0, fraction * model_->mass() * gaitos::kGravity / n);
  const Eigen::Vector3d f_base = orientation.conjugate() * f_world;
  for (std::size_t l = 0; l < model_->leg_count() && l < stance.size(); ++l) {
    if (!stance[l]) {
      continue;
    }
    const auto & chain = model_->leg(l).chain;
    const Eigen::VectorXd t = -chain.jacobian(q).topRows(3).transpose() * f_base;
    for (std::size_t k = 0; k < chain.dof(); ++k) {
      tau[static_cast<std::size_t>(chain.joint_indices()[k])] += t(static_cast<Eigen::Index>(k));
    }
  }
}

std::vector<Eigen::Isometry3d> LegIk::body_pose_feet(
  const std::vector<Eigen::Isometry3d> & feet, const Eigen::Vector3d & offset,
  const Eigen::Vector3d & rpy) const
{
  Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
  body.linear() = gaitos::rpy_to_matrix(rpy);
  body.translation() = offset;
  const Eigen::Isometry3d inv = body.inverse();
  std::vector<Eigen::Isometry3d> out;
  out.reserve(feet.size());
  for (const auto & f : feet) {
    out.push_back(inv * f);
  }
  return out;
}

void blend(const std::vector<double> & from, const std::vector<double> & to, double alpha,
  std::vector<double> & out)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  out.resize(to.size());
  for (std::size_t i = 0; i < to.size(); ++i) {
    const double f = i < from.size() ? from[i] : to[i];
    out[i] = f + alpha * (to[i] - f);
  }
}

}  // namespace gaitos_controllers
