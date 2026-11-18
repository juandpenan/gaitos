//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_dynamics/ConvexMpc.hpp"

#include "gaitos_common/Math.hpp"

namespace gaitos_dynamics
{

void ConvexMpc::configure(const Parameters & p, int legs)
{
  p_ = p;
  legs_ = legs;
  qp_.reset();
  u_.resize(0);
  qp_.settings().max_iterations = 400;
  qp_.settings().eps_abs = 1e-5;
  qp_.settings().eps_rel = 1e-5;
}

OsqpSolver::Result ConvexMpc::solve(
  const Vector13 & x0, const Eigen::VectorXd & x_ref, const std::vector<Eigen::Vector3d> & feet,
  const std::vector<std::vector<bool>> & contact, std::vector<Eigen::Vector3d> & forces)
{
  const int n = p_.horizon;
  const int nu = 3 * legs_;
  constexpr int nx = 13;
  const double yaw = x0(2);
  const Eigen::Matrix3d rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d inertia_world = rz * p_.inertia * rz.transpose();
  const Eigen::Matrix3d inv_inertia = inertia_world.inverse();

  // Continuous dynamics
  Eigen::Matrix<double, nx, nx> ac = Eigen::Matrix<double, nx, nx>::Zero();
  ac.block<3, 3>(0, 6) = rz.transpose();
  ac.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
  ac(11, 12) = 1.0;
  Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(nx, nu);
  for (int l = 0; l < legs_; ++l) {
    bc.block<3, 3>(6, 3 * l) = inv_inertia * gaitos::skew(feet[static_cast<std::size_t>(l)]);
    bc.block<3, 3>(9, 3 * l) = Eigen::Matrix3d::Identity() / p_.mass;
  }
  // Second order discretization
  const double dt = p_.dt;
  const Eigen::Matrix<double, nx, nx> ad = Eigen::Matrix<double, nx, nx>::Identity() + ac * dt +
    ac * ac * (dt * dt / 2.0);
  const Eigen::MatrixXd bd = (Eigen::Matrix<double, nx, nx>::Identity() * dt + ac * (dt * dt / 2.0)) * bc;

  // Condensed prediction: X = Aqp x0 + Bqp U
  Eigen::MatrixXd aqp(nx * n, nx);
  Eigen::MatrixXd bqp = Eigen::MatrixXd::Zero(nx * n, nu * n);
  std::vector<Eigen::MatrixXd> powers_b(static_cast<std::size_t>(n));   // A^k B
  Eigen::MatrixXd apow = Eigen::MatrixXd::Identity(nx, nx);
  for (int k = 0; k < n; ++k) {
    powers_b[static_cast<std::size_t>(k)] = apow * bd;
    apow = ad * apow;
    aqp.block(nx * k, 0, nx, nx) = apow;
  }
  for (int k = 0; k < n; ++k) {
    for (int j = 0; j <= k; ++j) {
      bqp.block(nx * k, nu * j, nx, nu) = powers_b[static_cast<std::size_t>(k - j)];
    }
  }
  Eigen::VectorXd lw(nx * n);
  for (int k = 0; k < n; ++k) {
    lw.segment<nx>(nx * k) = p_.weights;
  }
  // Forces are scaled (one unit = the weight shared by the legs) for a well conditioned QP
  const double scale = p_.mass * p_.gravity / std::max(1, legs_);
  const Eigen::MatrixXd bs = bqp * scale;
  const Eigen::MatrixXd h = 2.0 * (bs.transpose() * lw.asDiagonal() * bs +
    p_.force_weight * scale * scale * Eigen::MatrixXd::Identity(nu * n, nu * n));
  const Eigen::VectorXd g = 2.0 * bs.transpose() * lw.asDiagonal() * (aqp * x0 - x_ref);

  // Friction pyramid and normal force bounds
  const int rows = 5 * legs_ * n;
  Eigen::MatrixXd c = Eigen::MatrixXd::Zero(rows, nu * n);
  Eigen::VectorXd lo = Eigen::VectorXd::Constant(rows, -1e20);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(rows);
  for (int k = 0; k < n; ++k) {
    for (int l = 0; l < legs_; ++l) {
      const int r = 5 * (legs_ * k + l);
      const int col = nu * k + 3 * l;
      const double mu = p_.mu;
      c(r, col) = 1.0;
      c(r, col + 2) = -mu;
      c(r + 1, col) = -1.0;
      c(r + 1, col + 2) = -mu;
      c(r + 2, col + 1) = 1.0;
      c(r + 2, col + 2) = -mu;
      c(r + 3, col + 1) = -1.0;
      c(r + 3, col + 2) = -mu;
      c(r + 4, col + 2) = 1.0;
      const bool in_contact = contact[static_cast<std::size_t>(k)][static_cast<std::size_t>(l)];
      lo(r + 4) = in_contact ? p_.fz_min / scale : 0.0;
      hi(r + 4) = in_contact ? p_.fz_max / scale : 0.0;
    }
  }
  // OSQP warm starts from its previous solution
  const auto result = qp_.solve(h, g, c, lo, hi, u_);
  forces.resize(static_cast<std::size_t>(legs_));
  for (int l = 0; l < legs_; ++l) {
    forces[static_cast<std::size_t>(l)] = scale * u_.segment<3>(3 * l);
  }
  prediction_ = aqp * x0 + bs * u_;
  return result;
}

}  // namespace gaitos_dynamics
