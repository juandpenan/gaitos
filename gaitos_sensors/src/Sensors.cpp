//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_sensors/Sensors.hpp"

#include <algorithm>
#include <cmath>

#include "gaitos_common/Math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_sensors
{

namespace
{
void hysteresis(double force, double on, double off, std::size_t i, gaitos::ContactData & c)
{
  c.force[i] = force;
  if (force > on) {
    c.contact[i] = true;
  } else if (force < off) {
    c.contact[i] = false;
  }
  c.probability[i] = on > off ? std::clamp((force - off) / (on - off), 0.0, 1.0) :
    (c.contact[i] ? 1.0 : 0.0);
}
}  // namespace

bool ImuPassthrough::on_initialize()
{
  mounting_ = gaitos::rpy_to_matrix(params().get_vector3("mounting_rpy", Eigen::Vector3d::Zero()));
  gyro_bias_ = params().get_vector3("gyro_bias", Eigen::Vector3d::Zero());
  gyro_cutoff_ = params().get<double>("gyro_cutoff", 0.0);
  return true;
}

bool ImuPassthrough::update(const gaitos::ImuData & raw, double dt, gaitos::ImuData & imu)
{
  imu = raw;
  // Base orientation: R_wb = R_wi * R_bi^T
  imu.orientation = Eigen::Quaterniond(raw.orientation.normalized().toRotationMatrix() *
    mounting_.transpose());
  const Eigen::Vector3d gyro = mounting_ * (raw.angular_velocity - gyro_bias_);
  if (gyro_cutoff_ > 0.0 && !first_ && dt > 0.0) {
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * gyro_cutoff_ * dt);
    gyro_filtered_ += alpha * (gyro - gyro_filtered_);
  } else {
    gyro_filtered_ = gyro;
  }
  first_ = false;
  imu.angular_velocity = gyro_filtered_;
  imu.linear_acceleration = mounting_ * raw.linear_acceleration;
  return std::isfinite(imu.orientation.w()) && imu.angular_velocity.allFinite();
}

bool ComplementaryImu::on_initialize()
{
  kp_ = params().get<double>("kp", kp_);
  ki_ = params().get<double>("ki", ki_);
  use_device_orientation_ = params().get<bool>("use_device_orientation", use_device_orientation_);
  return ImuPassthrough::on_initialize();
}

void ComplementaryImu::reset()
{
  ImuPassthrough::reset();
  initialized_ = false;
  integral_.setZero();
}

bool ComplementaryImu::update(const gaitos::ImuData & raw, double dt, gaitos::ImuData & imu)
{
  if (!ImuPassthrough::update(raw, dt, imu)) {
    return false;
  }
  if (use_device_orientation_ && raw.has_orientation) {
    q_ = imu.orientation;
    initialized_ = true;
    return true;
  }
  const Eigen::Vector3d acc = imu.linear_acceleration;
  if (!initialized_) {
    // Level from the accelerometer, yaw 0
    if (acc.norm() < 1e-3) {
      return false;
    }
    q_ = Eigen::Quaterniond::FromTwoVectors(acc.normalized(), Eigen::Vector3d::UnitZ());
    initialized_ = true;
  }
  Eigen::Vector3d omega = imu.angular_velocity;
  const double acc_norm = acc.norm();
  // Correct only when the acceleration is close to gravity
  if (acc_norm > 0.5 * gaitos::kGravity && acc_norm < 1.5 * gaitos::kGravity) {
    const Eigen::Vector3d up_measured = acc / acc_norm;
    const Eigen::Vector3d up_estimated = q_.conjugate() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d error = up_measured.cross(up_estimated);
    integral_ += ki_ * error * dt;
    omega += kp_ * error + integral_;
  }
  const Eigen::Vector3d dtheta = omega * dt;
  const double angle = dtheta.norm();
  if (angle > 1e-12) {
    q_ = q_ * Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle));
  }
  q_.normalize();
  imu.orientation = q_;
  imu.has_orientation = true;
  return true;
}

bool ForceThresholdContact::on_initialize()
{
  on_ = params().get<double>("on_threshold", on_);
  off_ = params().get<double>("off_threshold", off_);
  assume_contact_ = params().get<bool>("assume_contact", assume_contact_);
  scale_ = params().get<std::vector<double>>("scale", {});
  return on_ >= off_;
}

bool ForceThresholdContact::update(const gaitos::RobotState & state, gaitos::ContactData & contacts)
{
  const auto n = contacts.size();
  if (state.foot_force_raw.size() != n) {
    for (std::size_t i = 0; i < n; ++i) {
      contacts.contact[i] = assume_contact_;
      contacts.probability[i] = assume_contact_ ? 1.0 : 0.0;
      contacts.force[i] = 0.0;
    }
    return assume_contact_;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const double s = i < scale_.size() ? scale_[i] : 1.0;
    hysteresis(s * state.foot_force_raw[i], on_, off_, i, contacts);
  }
  return true;
}

bool TorqueContact::on_initialize()
{
  on_ = params().get<double>("on_threshold", on_);
  off_ = params().get<double>("off_threshold", off_);
  filter_ = std::clamp(params().get<double>("filter", filter_), 0.0, 0.99);
  force_.assign(model().leg_count(), 0.0);
  return on_ >= off_;
}

bool TorqueContact::update(const gaitos::RobotState & state, gaitos::ContactData & contacts)
{
  const Eigen::Matrix3d r = state.imu.orientation.toRotationMatrix();
  for (std::size_t l = 0; l < model().leg_count(); ++l) {
    const auto & chain = model().leg(l).chain;
    const Eigen::MatrixXd jv = chain.jacobian(state.joints.position).topRows(3);
    Eigen::VectorXd tau(static_cast<Eigen::Index>(chain.dof()));
    for (std::size_t k = 0; k < chain.dof(); ++k) {
      tau(static_cast<Eigen::Index>(k)) =
        state.joints.effort[static_cast<std::size_t>(chain.joint_indices()[k])];
    }
    // tau = -J^T F  ->  F = -(J^T)^+ tau  (least squares, damped)
    const Eigen::Matrix3d jjt = jv * jv.transpose() + 1e-6 * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d f = -jjt.ldlt().solve(jv * tau);
    const double fz = (r * f).z();
    force_[l] = filter_ * force_[l] + (1.0 - filter_) * fz;
    hysteresis(force_[l], on_, off_, l, contacts);
  }
  return true;
}

}  // namespace gaitos_sensors

PLUGINLIB_EXPORT_CLASS(gaitos_sensors::ImuPassthrough, gaitos::ImuSensorBase)
PLUGINLIB_EXPORT_CLASS(gaitos_sensors::ComplementaryImu, gaitos::ImuSensorBase)
PLUGINLIB_EXPORT_CLASS(gaitos_sensors::ForceThresholdContact, gaitos::ContactSensorBase)
PLUGINLIB_EXPORT_CLASS(gaitos_sensors::TorqueContact, gaitos::ContactSensorBase)
