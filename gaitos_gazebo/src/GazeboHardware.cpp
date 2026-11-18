//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_gazebo/GazeboHardware.hpp"

namespace gaitos_gazebo
{

bool GazeboHardware::on_initialize()
{
  const auto n = model().joint_count();
  input_.joints.resize(n);
  input_.foot_force.assign(model().leg_count(), 0.0);
  command_.resize(n);
  return true;
}

bool GazeboHardware::read(gaitos::RobotState & state)
{
  state.joints = input_.joints;
  state.imu_raw = input_.imu;
  state.foot_force_raw = input_.foot_force;
  if (input_.has_ground_truth) {
    auto & gt = state.external_odometry["ground_truth"];
    gt.odometry = input_.ground_truth;
    gt.reception = input_.time;
    gt.seq = ++seq_;
  }
  return true;
}

bool GazeboHardware::write(const gaitos::JointCommand & command)
{
  command_ = command;
  return true;
}

}  // namespace gaitos_gazebo
