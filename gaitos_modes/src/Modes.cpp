//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_modes/Modes.hpp"

#include <algorithm>
#include <cmath>

#include "gaitos_common/Math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_modes
{

using gaitos::ModeStatus;
using gaitos::Params;

// ------------------------------------------------------------------------------------------

ModeStatus ZeroTorqueMode::update(const gaitos::RobotState & state, gaitos::ControlReference & reference)
{
  reference.joints.set_zero();
  reference.joints.position = state.joints.position;
  return ModeStatus::RUNNING;
}

// ------------------------------------------------------------------------------------------

bool DampMode::on_initialize()
{
  gains_ = gaitos::read_gains(params().sub("gains"), model(), 0.0, 2.0);
  return true;
}

ModeStatus DampMode::update(const gaitos::RobotState & state, gaitos::ControlReference & reference)
{
  reference.joints.set_zero();
  reference.joints.position = state.joints.position;
  reference.joints.kd = gains_.kd;
  return ModeStatus::RUNNING;
}

// ------------------------------------------------------------------------------------------

bool PostureMode::on_initialize()
{
  gains_ = gaitos::read_gains(params().sub("gains"), model(), 40.0, 2.0);
  auto add = [this](const std::string & pose, double duration) {
      steps_.push_back({pose, model().pose(pose), std::max(duration, 1e-3)});
    };
  const auto poses = params().get<std::vector<std::string>>("poses", {});
  if (!poses.empty()) {
    const auto durations = params().get<std::vector<double>>("durations", {});
    for (std::size_t i = 0; i < poses.size(); ++i) {
      add(poses[i], i < durations.size() ? durations[i] : 1.0);
    }
  } else {
    add(params().require<std::string>("pose"), params().get<double>("duration", 1.0));
  }
  return !steps_.empty();
}

void PostureMode::enter(const gaitos::RobotState & state)
{
  start_ = state.joints.position;
  elapsed_ = 0.0;
  last_time_ = state.time;
  // Joints not in the poses keep their position at entry
  for (auto & step : steps_) {
    step.q = model().pose(step.pose, start_);
  }
}

double PostureMode::progress() const
{
  double total = 0.0;
  for (const auto & s : steps_) {
    total += s.duration;
  }
  return total > 0.0 ? std::clamp(elapsed_ / total, 0.0, 1.0) : 1.0;
}

ModeStatus PostureMode::update(const gaitos::RobotState & state, gaitos::ControlReference & reference)
{
  elapsed_ += std::max(0.0, state.time - last_time_);
  last_time_ = state.time;
  double t = elapsed_;
  const std::vector<double> * from = &start_;
  const std::vector<double> * to = &steps_.back().q;
  double s = 1.0;
  for (const auto & step : steps_) {
    if (t < step.duration) {
      to = &step.q;
      s = gaitos::smoothstep(t / step.duration);
      break;
    }
    t -= step.duration;
    from = &step.q;
  }
  auto & j = reference.joints;
  for (std::size_t i = 0; i < j.size(); ++i) {
    j.position[i] = (*from)[i] + s * ((*to)[i] - (*from)[i]);
    j.velocity[i] = 0.0;
    j.effort[i] = 0.0;
    j.kp[i] = gains_.kp[i];
    j.kd[i] = gains_.kd[i];
  }
  return progress() >= 1.0 ? ModeStatus::FINISHED : ModeStatus::RUNNING;
}

// ------------------------------------------------------------------------------------------

bool BalanceStandMode::on_initialize()
{
  reset_on_enter_ = params().get<bool>("reset_pose_on_enter", reset_on_enter_);
  return model().leg_count() > 0;
}

void BalanceStandMode::enter(const gaitos::RobotState & state)
{
  goal_seq_ = state.pose_goal_seq;
  if (reset_on_enter_) {
    offset_.setZero();
    rpy_.setZero();
  }
}

ModeStatus BalanceStandMode::update(const gaitos::RobotState & state, gaitos::ControlReference & reference)
{
  if (state.pose_goal_seq != goal_seq_) {
    goal_seq_ = state.pose_goal_seq;
    // Relative to the current body posture
    const Eigen::Matrix3d r = gaitos::rpy_to_matrix(rpy_);
    offset_ += r * state.pose_goal.position;
    rpy_ = gaitos::matrix_to_rpy(r * state.pose_goal.orientation.toRotationMatrix());
  }
  reference.body_offset = offset_;
  reference.body_rpy = rpy_;
  reference.linear_velocity.setZero();
  reference.angular_velocity.setZero();
  return ModeStatus::RUNNING;
}

// ------------------------------------------------------------------------------------------

bool LocomotionMode::on_initialize()
{
  command_timeout_ = params().get<double>("command_timeout", command_timeout_);
  goal_gain_ = params().get_vector3("goal_gain", goal_gain_);
  goal_max_ = params().get_vector3("goal_max_velocity", goal_max_);
  const auto tol = params().get<std::vector<double>>("goal_tolerance", {0.05, 0.05});
  if (tol.size() == 2) {
    goal_tolerance_ = {tol[0], tol[1]};
  }
  gait_ = params().get<std::string>("gait", "");
  body_offset_ = params().get_vector3("body_offset", body_offset_);
  body_rpy_ = params().get_vector3("body_rpy", body_rpy_);
  return model().leg_count() > 0;
}

gaitos::CommandResult LocomotionMode::can_enter(const gaitos::RobotState & state) const
{
  if (!state.base.valid) {
    return gaitos::CommandResult::fail("no valid state estimate");
  }
  return gaitos::CommandResult::ok();
}

void LocomotionMode::enter(const gaitos::RobotState & state)
{
  goal_seq_ = state.pose_goal_seq;
  has_goal_ = false;
}

ModeStatus LocomotionMode::update(const gaitos::RobotState & state, gaitos::ControlReference & reference)
{
  if (!state.base.valid) {
    logger().error() << "State estimate lost";
    return ModeStatus::FAILED;
  }
  const double yaw = gaitos::yaw_of(state.base.orientation);
  const Eigen::Matrix3d yaw_rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  if (state.pose_goal_seq != goal_seq_) {
    goal_seq_ = state.pose_goal_seq;
    // Goal relative to the body (heading frame) -> odometry frame
    Eigen::Vector3d p = state.pose_goal.position;
    p.z() = 0.0;
    goal_position_ = state.base.position + yaw_rot * p;
    goal_yaw_ = gaitos::wrap_angle(yaw + gaitos::yaw_of(state.pose_goal.orientation));
    goal_stamp_ = state.pose_goal.stamp;
    has_goal_ = true;
    logger().info() << "Walking to [" << goal_position_.head<2>().transpose() << "], yaw " << goal_yaw_;
  }
  const auto & tw = state.twist_command;
  const bool twist_fresh = tw.stamp >= 0.0 && state.time - tw.stamp <= command_timeout_;
  if (has_goal_ && twist_fresh && tw.stamp > goal_stamp_) {
    has_goal_ = false;   // a newer twist cancels the goal
  }
  reference.linear_velocity.setZero();
  reference.angular_velocity.setZero();
  if (has_goal_) {
    const Eigen::Vector3d e = yaw_rot.transpose() * (goal_position_ - state.base.position);
    const double e_yaw = gaitos::wrap_angle(goal_yaw_ - yaw);
    if (e.head<2>().norm() < goal_tolerance_(0) && std::abs(e_yaw) < goal_tolerance_(1)) {
      has_goal_ = false;
      logger().info() << "Goal reached";
    } else {
      reference.linear_velocity.x() = std::clamp(goal_gain_.x() * e.x(), -goal_max_.x(), goal_max_.x());
      reference.linear_velocity.y() = std::clamp(goal_gain_.y() * e.y(), -goal_max_.y(), goal_max_.y());
      reference.angular_velocity.z() = std::clamp(goal_gain_.z() * e_yaw, -goal_max_.z(), goal_max_.z());
    }
  } else if (twist_fresh) {
    reference.linear_velocity = tw.linear;
    reference.angular_velocity = tw.angular;
  }
  reference.gait = gait_;
  reference.body_offset = body_offset_;
  reference.body_rpy = body_rpy_;
  return ModeStatus::RUNNING;
}

}  // namespace gaitos_modes

PLUGINLIB_EXPORT_CLASS(gaitos_modes::ZeroTorqueMode, gaitos::OperationModeBase)
PLUGINLIB_EXPORT_CLASS(gaitos_modes::DampMode, gaitos::OperationModeBase)
PLUGINLIB_EXPORT_CLASS(gaitos_modes::PostureMode, gaitos::OperationModeBase)
PLUGINLIB_EXPORT_CLASS(gaitos_modes::BalanceStandMode, gaitos::OperationModeBase)
PLUGINLIB_EXPORT_CLASS(gaitos_modes::LocomotionMode, gaitos::OperationModeBase)
