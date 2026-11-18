//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_dynamics/LipmPreview.hpp"

#include <stdexcept>

namespace gaitos_dynamics
{

void LipmPreview::configure(double dt, double com_height, double preview_time, double qe, double r, double gravity)
{
  if (dt <= 0.0 || com_height <= 0.0 || preview_time <= 0.0) {
    throw std::invalid_argument("LipmPreview: dt, com_height and preview_time must be positive");
  }
  dt_ = dt;
  zc_ = com_height;
  g_ = gravity;
  a_ << 1.0, dt, dt * dt / 2.0,
    0.0, 1.0, dt,
    0.0, 0.0, 1.0;
  b_ << dt * dt * dt / 6.0, dt * dt / 2.0, dt;
  c_ << 1.0, 0.0, -zc_ / g_;

  // Augmented system [e_sum; x] (Katayama): X(k+1) = At X(k) + Bt u(k) - It p_ref(k+1)
  Eigen::Matrix4d at = Eigen::Matrix4d::Zero();
  at(0, 0) = 1.0;
  at.block<1, 3>(0, 1) = c_ * a_;
  at.block<3, 3>(1, 1) = a_;
  Eigen::Vector4d bt;
  bt(0) = c_ * b_;
  bt.tail<3>() = b_;
  Eigen::Vector4d it = Eigen::Vector4d::Zero();
  it(0) = 1.0;
  Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
  q(0, 0) = qe;

  // Discrete algebraic Riccati equation by iteration
  Eigen::Matrix4d p = q;
  for (int k = 0; k < 20000; ++k) {
    const double s = r + bt.dot(p * bt);
    const Eigen::Matrix4d next = at.transpose() * p * at -
      (at.transpose() * p * bt) * (bt.transpose() * p * at) / s + q;
    const double diff = (next - p).cwiseAbs().maxCoeff();
    p = next;
    if (diff < 1e-12 * (1.0 + p.cwiseAbs().maxCoeff())) {
      break;
    }
  }
  const double s = r + bt.dot(p * bt);
  const Eigen::RowVector4d k = (bt.transpose() * p * at) / s;
  gi_ = k(0);
  gx_ = k.tail<3>();
  const Eigen::Matrix4d ac = at - bt * k;
  const int n = static_cast<int>(preview_time / dt);
  gp_.assign(static_cast<std::size_t>(n), 0.0);
  Eigen::Vector4d xx = -ac.transpose() * p * it;
  gp_[0] = -gi_;
  for (int j = 1; j < n; ++j) {
    gp_[static_cast<std::size_t>(j)] = bt.dot(xx) / s;
    xx = ac.transpose() * xx;
  }
}

void LipmPreview::reset(const Eigen::Vector2d & com)
{
  x_ = Eigen::Vector3d(com.x(), 0.0, 0.0);
  y_ = Eigen::Vector3d(com.y(), 0.0, 0.0);
  integral_.setZero();
}

Eigen::Vector2d LipmPreview::zmp() const
{
  return {c_ * x_, c_ * y_};
}

void LipmPreview::step(const std::vector<Eigen::Vector2d> & zmp_ref)
{
  if (zmp_ref.empty()) {
    return;
  }
  auto ref = [&zmp_ref](std::size_t j) -> const Eigen::Vector2d & {
      return zmp_ref[std::min(j, zmp_ref.size() - 1)];
    };
  integral_ += zmp() - ref(0);
  Eigen::Vector2d preview = Eigen::Vector2d::Zero();
  for (std::size_t j = 0; j < gp_.size(); ++j) {
    preview += gp_[j] * ref(j + 1);
  }
  const double ux = -gi_ * integral_.x() - gx_ * x_ - preview.x();
  const double uy = -gi_ * integral_.y() - gx_ * y_ - preview.y();
  x_ = a_ * x_ + b_ * ux;
  y_ = a_ * y_ + b_ * uy;
}

void LipmPreview::correct(
  const Eigen::Vector2d & com, const Eigen::Vector2d & com_vel, double gain_p, double gain_v)
{
  x_(0) += gain_p * (com.x() - x_(0));
  y_(0) += gain_p * (com.y() - y_(0));
  x_(1) += gain_v * (com_vel.x() - x_(1));
  y_(1) += gain_v * (com_vel.y() - y_(1));
}

}  // namespace gaitos_dynamics
