//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_system/Safety.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace gaitos
{

void SafetyMonitor::configure(const Params & limits, RobotModelPtr model)
{
  model_ = std::move(model);
  enabled_ = limits.get<bool>("enabled", enabled_);
  position_margin_ = limits.get<double>("position_margin", position_margin_);
  velocity_scale_ = limits.get<double>("velocity_scale", velocity_scale_);
  torque_scale_ = limits.get<double>("torque_scale", torque_scale_);
  cycles_ = std::max(1, limits.get<int>("cycles", cycles_));
  count_.assign(model_->joint_count(), 0);
}

std::string SafetyMonitor::check(const JointState & joints)
{
  if (!enabled_ || !model_) {
    return "";
  }
  for (std::size_t i = 0; i < model_->joint_count() && i < joints.size(); ++i) {
    const auto & j = model_->joint(i);
    std::ostringstream why;
    const double q = joints.position[i];
    if (position_margin_ >= 0.0 && (q < j.lower - position_margin_ || q > j.upper + position_margin_)) {
      why << "position " << q << " outside [" << j.lower << ", " << j.upper << "]";
    } else if (velocity_scale_ > 0.0 && j.velocity > 0.0 &&
      std::abs(joints.velocity[i]) > velocity_scale_ * j.velocity)
    {
      why << "velocity " << joints.velocity[i] << " over " << velocity_scale_ * j.velocity;
    } else if (torque_scale_ > 0.0 && j.effort > 0.0 &&
      std::abs(joints.effort[i]) > torque_scale_ * j.effort)
    {
      why << "torque " << joints.effort[i] << " over " << torque_scale_ * j.effort;
    }
    if (why.str().empty()) {
      count_[i] = 0;
      continue;
    }
    if (++count_[i] >= cycles_) {
      count_[i] = 0;
      return "joint " + j.name + ": " + why.str();
    }
  }
  return "";
}

void Teleop::configure(const Params & teleop)
{
  enabled_ = teleop.get<bool>("enabled", false);
  modes_.clear();
  if (!enabled_) {
    return;
  }
  const auto estop = teleop.get<std::string>("estop", "L2+B");
  estop_ = parse_buttons(estop);
  if (estop_ == 0) {
    throw std::runtime_error("teleop.estop: unknown buttons '" + estop + "'");
  }
  // Parallel lists: buttons[i] (e.g. "L2+A") requests modes[i]
  const auto buttons = teleop.get<std::vector<std::string>>("buttons", {});
  const auto modes = teleop.get<std::vector<std::string>>("modes", {});
  if (buttons.size() != modes.size()) {
    throw std::runtime_error("teleop.buttons and teleop.modes must have the same size");
  }
  for (std::size_t i = 0; i < buttons.size(); ++i) {
    const uint32_t mask = parse_buttons(buttons[i]);
    if (mask == 0) {
      throw std::runtime_error("teleop.buttons: unknown buttons '" + buttons[i] + "'");
    }
    modes_.emplace_back(mask, modes[i]);
  }
  // Longer combinations first, so that "L2+A" wins over "A"
  std::stable_sort(modes_.begin(), modes_.end(), [](const auto & a, const auto & b) {
      return __builtin_popcount(a.first) > __builtin_popcount(b.first);
    });
  scale_ = teleop.get_vector3("twist_scale", scale_);
  deadzone_ = teleop.get<double>("deadzone", deadzone_);
  timeout_ = teleop.get<double>("timeout", timeout_);
}

Teleop::Actions Teleop::update(const RemoteInput & remote, double now)
{
  Actions a;
  if (!enabled_ || !remote.valid || (timeout_ > 0.0 && now - remote.stamp > timeout_)) {
    previous_ = 0;
    if (sticks_active_) {   // stop when the remote is lost
      sticks_active_ = false;
      a.twist = true;
    }
    return a;
  }
  const uint32_t pressed = remote.buttons & ~previous_;
  if ((remote.buttons & estop_) == estop_) {
    a.estop = true;
  } else if (pressed) {
    for (const auto & [mask, mode] : modes_) {
      if ((remote.buttons & mask) == mask && (pressed & mask)) {
        a.mode = mode;
        break;
      }
    }
  }
  previous_ = remote.buttons;
  auto dz = [this](double v) {return std::abs(v) < deadzone_ ? 0.0 : v;};
  const Eigen::Vector3d cmd(dz(remote.left.y()) * scale_.x(), -dz(remote.left.x()) * scale_.y(),
    -dz(remote.right.x()) * scale_.z());
  if (cmd.norm() > 0.0) {
    sticks_active_ = true;
    a.twist = true;
    a.linear = Eigen::Vector3d(cmd.x(), cmd.y(), 0.0);
    a.angular = Eigen::Vector3d(0.0, 0.0, cmd.z());
  } else if (sticks_active_) {
    sticks_active_ = false;   // sticks released: one zero twist
    a.twist = true;
  }
  return a;
}

}  // namespace gaitos
