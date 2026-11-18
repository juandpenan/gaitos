//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_common/Types.hpp"
#include "gaitos_common/RobotState.hpp"

#include <algorithm>
#include <map>
#include <sstream>

namespace gaitos
{

void JointState::resize(std::size_t n)
{
  position.assign(n, 0.0);
  velocity.assign(n, 0.0);
  effort.assign(n, 0.0);
}

void JointCommand::resize(std::size_t n)
{
  position.assign(n, 0.0);
  velocity.assign(n, 0.0);
  effort.assign(n, 0.0);
  kp.assign(n, 0.0);
  kd.assign(n, 0.0);
}

void JointCommand::set_zero()
{
  std::fill(position.begin(), position.end(), 0.0);
  std::fill(velocity.begin(), velocity.end(), 0.0);
  std::fill(effort.begin(), effort.end(), 0.0);
  std::fill(kp.begin(), kp.end(), 0.0);
  std::fill(kd.begin(), kd.end(), 0.0);
}

void ContactData::resize(std::size_t n)
{
  force.assign(n, 0.0);
  contact.assign(n, false);
  probability.assign(n, 0.0);
}

std::size_t ContactData::count() const
{
  return static_cast<std::size_t>(std::count(contact.begin(), contact.end(), true));
}

Odometry BaseState::to_odometry() const
{
  Odometry o;
  o.stamp = stamp;
  o.position = position;
  o.orientation = orientation;
  o.linear_velocity = linear_velocity_body();
  o.angular_velocity = angular_velocity;
  return o;
}

uint32_t parse_buttons(const std::string & combo)
{
  static const std::map<std::string, uint32_t> names{
    {"R1", RemoteButton::R1}, {"L1", RemoteButton::L1}, {"START", RemoteButton::START},
    {"SELECT", RemoteButton::SELECT}, {"R2", RemoteButton::R2}, {"L2", RemoteButton::L2},
    {"F1", RemoteButton::F1}, {"F2", RemoteButton::F2}, {"A", RemoteButton::A},
    {"B", RemoteButton::B}, {"X", RemoteButton::X}, {"Y", RemoteButton::Y},
    {"UP", RemoteButton::UP}, {"RIGHT", RemoteButton::RIGHT}, {"DOWN", RemoteButton::DOWN},
    {"LEFT", RemoteButton::LEFT}};
  uint32_t mask = 0;
  std::stringstream ss(combo);
  std::string part;
  while (std::getline(ss, part, '+')) {
    part.erase(std::remove_if(part.begin(), part.end(), ::isspace), part.end());
    std::transform(part.begin(), part.end(), part.begin(), ::toupper);
    const auto it = names.find(part);
    if (it == names.end()) {
      return 0;
    }
    mask |= it->second;
  }
  return mask;
}

void RobotState::resize(std::size_t n_joints, std::size_t n_feet)
{
  joints.resize(n_joints);
  command.resize(n_joints);
  motor_command.resize(n_joints);
  reference.joints.resize(n_joints);
  contacts.resize(n_feet);
}

}  // namespace gaitos
