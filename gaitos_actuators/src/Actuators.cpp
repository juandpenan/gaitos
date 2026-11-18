//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_actuators/Actuators.hpp"

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace gaitos_actuators
{

bool LimitsBase::on_initialize()
{
  const double scale = params().get<double>("torque_scale", 1.0);
  const double max_torque = params().get<double>("max_torque", 0.0);
  position_margin_ = params().get<double>("position_margin", 0.0);
  max_kp_ = params().get<double>("max_kp", max_kp_);
  max_kd_ = params().get<double>("max_kd", max_kd_);
  torque_limit_.assign(model().joint_count(), 0.0);
  for (std::size_t i = 0; i < model().joint_count(); ++i) {
    torque_limit_[i] = max_torque > 0.0 ? max_torque : scale * model().joint(i).effort;
  }
  return true;
}

double LimitsBase::clamp_position(std::size_t i, double q) const
{
  const auto & j = model().joint(i);
  const double lo = j.lower + position_margin_;
  const double hi = j.upper - position_margin_;
  return lo < hi ? std::clamp(q, lo, hi) : q;
}

double LimitsBase::clamp_torque(std::size_t i, double tau) const
{
  const double l = torque_limit_[i];
  return l > 0.0 ? std::clamp(tau, -l, l) : tau;
}

void PdPassthrough::update(
  const gaitos::JointState & measured, const gaitos::JointCommand & command,
  gaitos::JointCommand & output)
{
  (void)measured;
  for (const auto i : joints()) {
    output.position[i] = clamp_position(i, command.position[i]);
    output.velocity[i] = command.velocity[i];
    output.kp[i] = std::clamp(command.kp[i], 0.0, max_kp_);
    output.kd[i] = std::clamp(command.kd[i], 0.0, max_kd_);
    output.effort[i] = clamp_torque(i, command.effort[i]);
  }
}

void TorquePd::update(
  const gaitos::JointState & measured, const gaitos::JointCommand & command,
  gaitos::JointCommand & output)
{
  for (const auto i : joints()) {
    const double kp = std::clamp(command.kp[i], 0.0, max_kp_);
    const double kd = std::clamp(command.kd[i], 0.0, max_kd_);
    const double tau = kp * (clamp_position(i, command.position[i]) - measured.position[i]) +
      kd * (command.velocity[i] - measured.velocity[i]) + command.effort[i];
    output.position[i] = measured.position[i];
    output.velocity[i] = 0.0;
    output.kp[i] = 0.0;
    output.kd[i] = 0.0;
    output.effort[i] = clamp_torque(i, tau);
  }
}

bool PositionServo::on_initialize()
{
  stiffness_ = params().get<double>("stiffness", stiffness_);
  use_command_kp_ = params().get<bool>("use_command_kp", use_command_kp_);
  servo_kp_ = params().get<double>("servo.kp", servo_kp_);
  servo_kd_ = params().get<double>("servo.kd", servo_kd_);
  return LimitsBase::on_initialize();
}

void PositionServo::update(
  const gaitos::JointState & measured, const gaitos::JointCommand & command,
  gaitos::JointCommand & output)
{
  for (const auto i : joints()) {
    const bool passive = command.kp[i] <= 0.0;
    output.position[i] = passive ? measured.position[i] : clamp_position(i, command.position[i]);
    output.velocity[i] = 0.0;
    const double stiffness = passive ? 0.0 :
      (use_command_kp_ ? std::min(command.kp[i], max_kp_) : stiffness_);
    if (servo_kp_ > 0.0) {   // simulated servo: a PD scaled by the stiffness
      output.kp[i] = stiffness * servo_kp_;
      output.kd[i] = stiffness * servo_kd_;
    } else {
      output.kp[i] = stiffness;
      output.kd[i] = 0.0;
    }
    output.effort[i] = 0.0;
  }
}

}  // namespace gaitos_actuators

PLUGINLIB_EXPORT_CLASS(gaitos_actuators::PdPassthrough, gaitos::JointActuatorBase)
PLUGINLIB_EXPORT_CLASS(gaitos_actuators::TorquePd, gaitos::JointActuatorBase)
PLUGINLIB_EXPORT_CLASS(gaitos_actuators::PositionServo, gaitos::JointActuatorBase)
