//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_estimators/Estimators.hpp"

#include <algorithm>
#include <cmath>

#include "gaitos_common/Math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_estimators
{

using gaitos::Params;

void leg_kinematics(
  const gaitos::RobotModel & model, const gaitos::JointState & joints,
  std::vector<LegMeasurement> & legs)
{
  legs.resize(model.leg_count());
  for (std::size_t l = 0; l < model.leg_count(); ++l) {
    const auto & chain = model.leg(l).chain;
    Eigen::Isometry3d tip;
    const Eigen::MatrixXd j = chain.jacobian(joints.position, &tip);
    Eigen::VectorXd dq(static_cast<Eigen::Index>(chain.dof()));
    for (std::size_t k = 0; k < chain.dof(); ++k) {
      dq(static_cast<Eigen::Index>(k)) =
        joints.velocity[static_cast<std::size_t>(chain.joint_indices()[k])];
    }
    legs[l].position = tip.translation();
    legs[l].velocity = j.topRows(3) * dq;
  }
}

ExternalSource ExternalSource::from_params(const std::string & name, const Params & p)
{
  ExternalSource s;
  s.name = name;
  s.position = p.get<bool>("position", s.position);
  s.velocity = p.get<bool>("velocity", s.velocity);
  s.yaw = p.get<bool>("yaw", s.yaw);
  s.position_noise = p.get<double>("position_noise", s.position_noise);
  s.velocity_noise = p.get<double>("velocity_noise", s.velocity_noise);
  s.yaw_gain = p.get<double>("yaw_gain", s.yaw_gain);
  s.align = p.get<bool>("align", s.align);
  s.timeout = p.get<double>("timeout", s.timeout);
  s.gate = p.get<double>("gate", s.gate);
  return s;
}

double ExternalSource::yaw_to_world(double yaw) const
{
  return gaitos::wrap_angle(yaw + std::atan2(align_rotation(1, 0), align_rotation(0, 0)));
}

// ------------------------------------------------------------------------------------------
// KalmanEstimator

bool KalmanEstimator::on_initialize()
{
  const Params pn = params().sub("process_noise");
  q_position_ = pn.get<double>("position", q_position_);
  q_velocity_ = pn.get<double>("velocity", q_velocity_);
  q_foot_ = pn.get<double>("foot", q_foot_);
  q_foot_swing_ = pn.get<double>("foot_swing", q_foot_swing_);
  const Params mn = params().sub("measurement_noise");
  r_foot_position_ = mn.get<double>("foot_position", r_foot_position_);
  r_foot_velocity_ = mn.get<double>("foot_velocity", r_foot_velocity_);
  r_foot_height_ = mn.get<double>("foot_height", r_foot_height_);
  flat_ground_ = params().get<bool>("flat_ground", flat_ground_);
  ground_height_ = params().get<double>("ground_height", ground_height_);
  initialize_from_ = params().get<std::string>("initialize_from", "");
  const Params ext = params().sub("external");
  for (const auto & name : ext.keys()) {
    sources_[name] = ExternalSource::from_params(name, ext.sub(name));
  }
  legs_ = model().leg_count();
  const auto n = static_cast<Eigen::Index>(6 + 3 * legs_);
  x_ = Eigen::VectorXd::Zero(n);
  p_ = Eigen::MatrixXd::Identity(n, n);
  if (legs_ == 0) {
    logger().warn() << "The robot has no legs: only the IMU and the external sources are used";
  }
  return true;
}

void KalmanEstimator::initialize(const gaitos::RobotState & state, const Eigen::Matrix3d & r_imu)
{
  yaw_offset_ = 0.0;
  Eigen::Matrix3d r = r_imu;
  Eigen::Vector3d p(0.0, 0.0, 0.0);
  bool from_source = false;
  if (!initialize_from_.empty()) {
    const auto it = state.external_odometry.find(initialize_from_);
    if (it != state.external_odometry.end() && it->second.seq > 0) {
      const auto & o = it->second.odometry;
      yaw_offset_ = gaitos::wrap_angle(gaitos::yaw_of(o.orientation) -
          gaitos::yaw_of(Eigen::Quaterniond(r_imu)));
      r = Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ()).toRotationMatrix() * r_imu;
      p = o.position;
      from_source = true;
      // The source is the estimator frame
      auto s = sources_.find(initialize_from_);
      if (s != sources_.end()) {
        s->second.aligned = true;
      }
    }
  }
  if (!from_source && legs_ > 0) {
    double z = 0.0;
    for (const auto & leg : leg_meas_) {
      z += (r * leg.position).z();
    }
    p.z() = ground_height_ - z / static_cast<double>(legs_);
  }
  x_.setZero();
  x_.segment<3>(0) = p;
  for (std::size_t l = 0; l < legs_; ++l) {
    x_.segment<3>(static_cast<Eigen::Index>(6 + 3 * l)) = p + r * leg_meas_[l].position;
  }
  p_.setIdentity();
  p_.block<3, 3>(0, 0) *= 1e-4;
  p_.block<3, 3>(3, 3) *= 1e-2;
  for (std::size_t l = 0; l < legs_; ++l) {
    p_.block<3, 3>(static_cast<Eigen::Index>(6 + 3 * l), static_cast<Eigen::Index>(6 + 3 * l)) *= 1e-3;
  }
  initialized_ = true;
  logger().info() << "Initialized at [" << p.transpose() << "]" <<
  (from_source ? " from " + initialize_from_ : std::string(""));
}

void KalmanEstimator::predict(
  const Eigen::Vector3d & acc_world, double dt, const gaitos::ContactData & contacts)
{
  x_.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * acc_world * dt * dt;
  x_.segment<3>(3) += acc_world * dt;
  // P = F P F^T + Q with F = [I dt I; 0 I] on the base block
  const auto n = x_.size();
  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(n, n);
  f.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  p_ = f * p_ * f.transpose();
  p_.block<3, 3>(0, 0).diagonal().array() += q_position_ * dt;
  p_.block<3, 3>(3, 3).diagonal().array() += q_velocity_ * dt;
  for (std::size_t l = 0; l < legs_; ++l) {
    const bool stance = l < contacts.size() && contacts.contact[l];
    const auto k = static_cast<Eigen::Index>(6 + 3 * l);
    p_.block<3, 3>(k, k).diagonal().array() += (stance ? q_foot_ : q_foot_swing_) * dt;
  }
}

bool KalmanEstimator::correct(
  const Eigen::VectorXd & y, const Eigen::MatrixXd & h, const Eigen::VectorXd & r_diag, double gate)
{
  const Eigen::MatrixXd s = h * p_ * h.transpose() + Eigen::MatrixXd(r_diag.asDiagonal());
  const Eigen::VectorXd e = y - h * x_;
  const Eigen::LDLT<Eigen::MatrixXd> s_ldlt(s);
  if (gate > 0.0 && e.dot(s_ldlt.solve(e)) > gate) {
    return false;
  }
  const Eigen::MatrixXd k = p_ * h.transpose() * s_ldlt.solve(Eigen::MatrixXd::Identity(s.rows(), s.cols()));
  x_ += k * e;
  // Joseph form keeps P symmetric and positive
  const auto n = x_.size();
  const Eigen::MatrixXd ikh = Eigen::MatrixXd::Identity(n, n) - k * h;
  p_ = ikh * p_ * ikh.transpose() + k * r_diag.asDiagonal() * k.transpose();
  return true;
}

void KalmanEstimator::fuse_external(const gaitos::RobotState & state, Eigen::Matrix3d & r)
{
  const auto n = x_.size();
  for (auto & [name, src] : sources_) {
    const auto it = state.external_odometry.find(name);
    if (it == state.external_odometry.end() || it->second.seq <= src.last_seq) {
      continue;
    }
    src.last_seq = it->second.seq;
    if (state.time - it->second.reception > src.timeout) {
      continue;
    }
    const auto & o = it->second.odometry;
    const double yaw_est = gaitos::yaw_of(Eigen::Quaterniond(r));
    if (!src.aligned) {
      if (src.align) {
        const double dyaw = o.has_orientation ? gaitos::wrap_angle(yaw_est - gaitos::yaw_of(o.orientation)) : 0.0;
        src.align_rotation = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        src.align_translation = x_.segment<3>(0) - src.align_rotation * o.position;
      }
      src.aligned = true;
      logger().info() << "External odometry '" << name << "' aligned";
      continue;
    }
    if (src.position && o.has_position) {
      Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, n);
      h.block<3, 3>(0, 0).setIdentity();
      Eigen::VectorXd rd(3);
      for (int i = 0; i < 3; ++i) {
        rd(i) = o.position_variance(i) > 0.0 ? o.position_variance(i) : src.position_noise * src.position_noise;
      }
      if (correct(src.to_world(o.position), h, rd, src.gate)) {
        ++src.accepted;
      } else {
        ++src.rejected;
      }
    }
    if (src.velocity && o.has_linear_velocity) {
      Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, n);
      h.block<3, 3>(0, 3).setIdentity();
      Eigen::VectorXd rd(3);
      for (int i = 0; i < 3; ++i) {
        rd(i) = o.velocity_variance(i) > 0.0 ? o.velocity_variance(i) : src.velocity_noise * src.velocity_noise;
      }
      correct(r * o.linear_velocity, h, rd, src.gate);
    }
    if (src.yaw && o.has_orientation) {
      const double err = gaitos::wrap_angle(src.yaw_to_world(gaitos::yaw_of(o.orientation)) - yaw_est);
      yaw_offset_ = gaitos::wrap_angle(yaw_offset_ + src.yaw_gain * err);
      r = Eigen::AngleAxisd(src.yaw_gain * err, Eigen::Vector3d::UnitZ()).toRotationMatrix() * r;
    }
  }
}

bool KalmanEstimator::update(const gaitos::RobotState & state, gaitos::BaseState & estimate)
{
  if (!state.imu.has_orientation) {
    return false;
  }
  leg_kinematics(model(), state.joints, leg_meas_);
  const Eigen::Matrix3d r_imu = state.imu.orientation.toRotationMatrix();
  if (!initialized_) {
    initialize(state, r_imu);
  }
  Eigen::Matrix3d r = Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ()).toRotationMatrix() * r_imu;
  const double dt = std::clamp(state.dt, 0.0, 0.05);
  const Eigen::Vector3d omega = state.imu.angular_velocity;
  const Eigen::Vector3d acc_world = r * state.imu.linear_acceleration -
    Eigen::Vector3d(0.0, 0.0, gaitos::kGravity);
  if (dt > 0.0) {
    predict(acc_world, dt, state.contacts);
  }

  // Leg kinematics: foot positions (always), foot velocity and height (stance)
  if (legs_ > 0) {
    const auto n = x_.size();
    const auto rows_per_leg = flat_ground_ ? 7 : 6;
    const auto m = static_cast<Eigen::Index>(rows_per_leg * legs_);
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(m, n);
    Eigen::VectorXd y(m);
    Eigen::VectorXd rd(m);
    for (std::size_t l = 0; l < legs_; ++l) {
      const auto row = static_cast<Eigen::Index>(rows_per_leg * l);
      const auto foot = static_cast<Eigen::Index>(6 + 3 * l);
      const bool stance = l < state.contacts.size() && state.contacts.contact[l];
      const auto & lm = leg_meas_[l];
      // f_l - p = R r_l
      h.block<3, 3>(row, 0) = -Eigen::Matrix3d::Identity();
      h.block<3, 3>(row, foot) = Eigen::Matrix3d::Identity();
      y.segment<3>(row) = r * lm.position;
      rd.segment<3>(row).setConstant(r_foot_position_ * r_foot_position_);
      // v = -R (J dq + w x r_l) when the foot does not slip
      h.block<3, 3>(row + 3, 3) = Eigen::Matrix3d::Identity();
      y.segment<3>(row + 3) = -r * (lm.velocity + omega.cross(lm.position));
      rd.segment<3>(row + 3).setConstant(stance ? r_foot_velocity_ * r_foot_velocity_ : r_swing_);
      if (flat_ground_) {
        h(row + 6, foot + 2) = 1.0;
        y(row + 6) = ground_height_;
        rd(row + 6) = stance ? r_foot_height_ * r_foot_height_ : r_swing_;
      }
    }
    correct(y, h, rd, 0.0);
  }
  fuse_external(state, r);

  estimate.position = x_.segment<3>(0);
  estimate.linear_velocity = x_.segment<3>(3);
  estimate.orientation = Eigen::Quaterniond(r).normalized();
  estimate.angular_velocity = omega;
  estimate.stamp = state.time;
  estimate.valid = x_.allFinite();
  if (!estimate.valid) {
    logger().error() << "Non finite state: resetting";
    initialized_ = false;
  }
  return estimate.valid;
}

// ------------------------------------------------------------------------------------------
// LegOdometryEstimator

bool LegOdometryEstimator::on_initialize()
{
  flat_ground_ = params().get<bool>("flat_ground", flat_ground_);
  filter_ = std::clamp(params().get<double>("velocity_filter", filter_), 0.0, 0.99);
  return model().leg_count() > 0;
}

bool LegOdometryEstimator::update(const gaitos::RobotState & state, gaitos::BaseState & estimate)
{
  if (!state.imu.has_orientation) {
    return false;
  }
  leg_kinematics(model(), state.joints, leg_meas_);
  const Eigen::Matrix3d r = state.imu.orientation.toRotationMatrix();
  const Eigen::Vector3d omega = state.imu.angular_velocity;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  double z = 0.0;
  std::size_t count = 0;
  for (std::size_t l = 0; l < leg_meas_.size(); ++l) {
    if (l < state.contacts.size() && state.contacts.contact[l]) {
      v += -r * (leg_meas_[l].velocity + omega.cross(leg_meas_[l].position));
      z += -(r * leg_meas_[l].position).z();
      ++count;
    }
  }
  if (!initialized_) {
    position_.setZero();
    velocity_.setZero();
    initialized_ = true;
  }
  const double dt = std::clamp(state.dt, 0.0, 0.05);
  if (count > 0) {
    v /= static_cast<double>(count);
    velocity_ = filter_ * velocity_ + (1.0 - filter_) * v;
  }
  position_ += velocity_ * dt;
  if (flat_ground_ && count > 0) {
    position_.z() = z / static_cast<double>(count);
  }
  estimate.position = position_;
  estimate.linear_velocity = velocity_;
  estimate.orientation = state.imu.orientation;
  estimate.angular_velocity = omega;
  estimate.stamp = state.time;
  estimate.valid = true;
  return true;
}

// ------------------------------------------------------------------------------------------
// ExternalOdometryEstimator

bool ExternalOdometryEstimator::on_initialize()
{
  source_ = params().get<std::string>("source", source_);
  use_imu_orientation_ = params().get<bool>("use_imu_orientation", use_imu_orientation_);
  timeout_ = params().get<double>("timeout", timeout_);
  return true;
}

bool ExternalOdometryEstimator::update(const gaitos::RobotState & state, gaitos::BaseState & estimate)
{
  const auto it = state.external_odometry.find(source_);
  if (it == state.external_odometry.end() || it->second.seq == 0 ||
    state.time - it->second.reception > timeout_)
  {
    estimate.valid = false;
    return false;
  }
  const bool fresh = it->second.seq != last_seq_;
  last_seq_ = it->second.seq;
  const auto & o = it->second.odometry;
  estimate.position = o.position;
  estimate.orientation = use_imu_orientation_ ? state.imu.orientation : o.orientation;
  estimate.linear_velocity = estimate.orientation * o.linear_velocity;
  estimate.angular_velocity = use_imu_orientation_ ? state.imu.angular_velocity : o.angular_velocity;
  estimate.stamp = state.time;
  estimate.valid = true;
  return fresh;
}

}  // namespace gaitos_estimators

PLUGINLIB_EXPORT_CLASS(gaitos_estimators::KalmanEstimator, gaitos::StateEstimatorBase)
PLUGINLIB_EXPORT_CLASS(gaitos_estimators::LegOdometryEstimator, gaitos::StateEstimatorBase)
PLUGINLIB_EXPORT_CLASS(gaitos_estimators::ExternalOdometryEstimator, gaitos::StateEstimatorBase)
