//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_hardware/FakeHardware.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace gaitos_hardware
{

bool FakeHardware::on_initialize()
{
  period_ = params().get<double>("period", period_);
  inertia_ = params().get<double>("joint_inertia", inertia_);
  damping_ = params().get<double>("joint_damping", damping_);
  foot_force_ = params().get<double>("foot_force", foot_force_);
  base_height_ = params().get<double>("base_height", base_height_);
  ground_truth_ = params().get<bool>("publish_ground_truth", ground_truth_);
  const auto n = model().joint_count();
  q_.assign(n, 0.0);
  dq_.assign(n, 0.0);
  tau_.assign(n, 0.0);
  command_.resize(n);
  const auto pose = params().get<std::string>("initial_pose", "");
  if (!pose.empty()) {
    q_ = model().pose(pose, q_);
  }
  model().clamp_to_limits(q_);
  return period_ > 0.0 && inertia_ > 0.0;
}

void FakeHardware::set_remote(const gaitos::RemoteInput & remote)
{
  std::lock_guard<std::mutex> lock(remote_mutex_);
  remote_ = remote;
  remote_.valid = true;
}

bool FakeHardware::read(gaitos::RobotState & state)
{
  const double stall = stall_.exchange(0.0);
  if (stall > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(stall));
  }
  {
    std::lock_guard<std::mutex> lock(remote_mutex_);
    state.remote = remote_;
    state.remote.stamp = time_;
  }
  state.joints.position = q_;
  state.joints.velocity = dq_;
  state.joints.effort = tau_;
  state.imu_raw = gaitos::ImuData();
  state.imu_raw.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  state.imu_raw.stamp = time_;
  state.foot_force_raw.assign(model().leg_count(), foot_force_);
  if (ground_truth_) {
    auto & gt = state.external_odometry["ground_truth"];
    gt.odometry = gaitos::Odometry();
    gt.odometry.stamp = time_;
    gt.odometry.position = Eigen::Vector3d(0.0, 0.0, base_height_);
    gt.reception = time_;
    gt.seq = ++seq_;
  }
  return true;
}

bool FakeHardware::write(const gaitos::JointCommand & command)
{
  command_ = command;
  ++writes_;
  for (std::size_t i = 0; i < q_.size(); ++i) {
    double tau = command.torque(i, q_[i], dq_[i]);
    const double limit = model().joint(i).effort;
    if (limit > 0.0) {
      tau = std::clamp(tau, -limit, limit);
    }
    tau_[i] = tau;
    // Semi-implicit Euler with a few substeps for stiff gains
    const int substeps = 4;
    const double h = period_ / substeps;
    for (int s = 0; s < substeps; ++s) {
      const double t = std::clamp(command.torque(i, q_[i], dq_[i]),
          limit > 0.0 ? -limit : -1e9, limit > 0.0 ? limit : 1e9);
      dq_[i] += h * (t - damping_ * dq_[i]) / inertia_;
      q_[i] += h * dq_[i];
      const auto & j = model().joint(i);
      if (q_[i] < j.lower || q_[i] > j.upper) {
        q_[i] = std::clamp(q_[i], j.lower, j.upper);
        dq_[i] = 0.0;
      }
    }
  }
  time_ += period_;
  return true;
}

}  // namespace gaitos_hardware

PLUGINLIB_EXPORT_CLASS(gaitos_hardware::FakeHardware, gaitos::HardwareInterface)
