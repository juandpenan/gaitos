//  Copyright 2026 Intelligent Robotics Lab (URJC)
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_unitree_sim/Dex3Hand.hpp"

#include <algorithm>
#include <cmath>

namespace gaitos_unitree_sim
{

Dex3Hand::Dex3Hand(
  const Vector & lower, const Vector & upper, double time_constant, double max_speed,
  double max_effort)
: lower_(lower), upper_(upper), time_constant_(std::max(1e-3, time_constant)),
  max_speed_(max_speed), max_effort_(max_effort)
{
  for (std::size_t i = 0; i < kMotors; ++i) {   // start at 0, or the nearest limit
    q_[i] = std::clamp(0.0, lower_[i], upper_[i]);
  }
  target_ = q_;
}

void Dex3Hand::command(const Vector & q, const Vector & kp, const Vector & kd)
{
  for (std::size_t i = 0; i < kMotors; ++i) {
    if (std::isfinite(q[i])) {
      target_[i] = std::clamp(q[i], lower_[i], upper_[i]);
    }
  }
  kp_ = kp;
  kd_ = kd;
}

void Dex3Hand::update(double dt)
{
  if (dt <= 0.0) {
    return;
  }
  for (std::size_t i = 0; i < kMotors; ++i) {
    if (kp_[i] <= 0.0) {   // no stiffness: the finger stays (no gravity on a kinematic hand)
      dq_[i] = 0.0;
      tau_[i] = 0.0;
      continue;
    }
    const double speed = std::clamp((target_[i] - q_[i]) / time_constant_, -max_speed_, max_speed_);
    const double previous = q_[i];
    q_[i] = std::clamp(q_[i] + speed * dt, lower_[i], upper_[i]);
    dq_[i] = (q_[i] - previous) / dt;
    tau_[i] = std::clamp(kp_[i] * (target_[i] - q_[i]) - kd_[i] * dq_[i], -max_effort_, max_effort_);
  }
}

}  // namespace gaitos_unitree_sim
