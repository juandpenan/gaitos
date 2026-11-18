//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_modes/ModeManager.hpp"

#include <algorithm>
#include <set>

#include "gaitos_common/Math.hpp"

namespace gaitos
{

void ModeManager::configure(const Params & config, RobotModelPtr model)
{
  modes_.clear();
  const Params modes = config.sub("modes");
  const Params safety = config.sub("safety");
  const auto fall_modes = safety.get<std::vector<std::string>>("fall_check_modes", {});
  for (const auto & name : modes.keys()) {
    const Params p = modes.sub(name);
    Entry e;
    e.mode = loader_.create(name, p, model);
    e.controller = e.mode->controller();
    e.next = e.mode->next();
    e.on_twist = p.get<std::string>("on_twist", "");
    e.allowed_from = e.mode->allowed_from();
    e.fall_check = p.get<bool>("fall_check", true);
    if (!fall_modes.empty()) {
      e.fall_check = std::find(fall_modes.begin(), fall_modes.end(), name) != fall_modes.end();
    }
    modes_[name] = std::move(e);
  }
  if (modes_.empty()) {
    throw std::runtime_error("No operation modes configured (modes)");
  }
  safe_mode_ = safety.get<std::string>("safe_mode", modes_.count("damp") ? "damp" : modes_.begin()->first);
  max_tilt_ = safety.get<double>("max_tilt", max_tilt_);
  if (!modes_.count(safe_mode_)) {
    throw std::runtime_error("Unknown safe mode " + safe_mode_);
  }
  modes_[safe_mode_].fall_check = false;
  for (const auto & [name, e] : modes_) {
    for (const auto & ref : {e.next, e.on_twist}) {
      if (!ref.empty() && !modes_.count(ref)) {
        throw std::runtime_error("Mode " + name + " references unknown mode " + ref);
      }
    }
  }
  current_ = config.get<std::string>("initial_mode", safe_mode_);
  if (!modes_.count(current_)) {
    throw std::runtime_error("Unknown initial mode " + current_);
  }
  current_.insert(0, "");   // entered on the first update
  entered_ = -1.0;
}

std::vector<std::string> ModeManager::modes() const
{
  std::vector<std::string> out;
  for (const auto & [name, e] : modes_) {
    (void)e;
    out.push_back(name);
  }
  return out;
}

std::vector<std::string> ModeManager::controllers() const
{
  std::set<std::string> c;
  for (const auto & [name, e] : modes_) {
    (void)name;
    c.insert(e.controller);
  }
  return {c.begin(), c.end()};
}

OperationModeBase * ModeManager::current_mode() const
{
  const auto it = modes_.find(current_);
  return it == modes_.end() ? nullptr : it->second.mode.get();
}

void ModeManager::switch_to(const std::string & name, const RobotState & state, const std::string & reason)
{
  const std::string from = current_;
  if (auto * m = current_mode(); m && entered_ >= 0.0) {
    m->exit(state);
  }
  current_ = name;
  entered_ = state.time;
  chain_failed_ = false;
  modes_.at(name).mode->enter(state);
  logger_.info() << "Mode " << (from.empty() ? "-" : from) << " -> " << name <<
    (reason.empty() ? "" : " (" + reason + ")");
  if (callback_) {
    callback_(from, name, reason);
  }
}

CommandResult ModeManager::request(const std::string & name, const RobotState & state)
{
  const auto it = modes_.find(name);
  if (it == modes_.end()) {
    return CommandResult::fail("unknown mode " + name);
  }
  const auto & allowed = it->second.allowed_from;
  if (!allowed.empty() && std::find(allowed.begin(), allowed.end(), current_) == allowed.end()) {
    return CommandResult::fail("mode " + name + " cannot be entered from " + current_);
  }
  const auto check = it->second.mode->can_enter(state);
  if (!check.success) {
    return CommandResult::fail("mode " + name + " rejected: " + check.message);
  }
  switch_to(name, state, "requested");
  return CommandResult::ok(name);
}

void ModeManager::force_safe(const RobotState & state, const std::string & reason)
{
  if (current_ != safe_mode_) {
    logger_.warn() << "Going to " << safe_mode_ << ": " << reason;
    switch_to(safe_mode_, state, reason);
  }
}

const std::string & ModeManager::update(RobotState & state, ControlReference & reference)
{
  if (entered_ < 0.0) {
    switch_to(current_, state, "initial mode");
  }
  auto & entry = modes_.at(current_);
  // Safety: fall detection
  if (entry.fall_check && max_tilt_ > 0.0 && tilt_angle(state.imu.orientation) > max_tilt_) {
    force_safe(state, "fall detected (tilt " + std::to_string(tilt_angle(state.imu.orientation)) + " rad)");
  }
  // Twist while standing: switch to the walking mode
  const auto & e = modes_.at(current_);
  if (!e.on_twist.empty() && state.twist_command.stamp > entered_ &&
    state.time - state.twist_command.stamp < 0.5 &&
    (state.twist_command.linear.norm() > 1e-3 || state.twist_command.angular.norm() > 1e-3))
  {
    const auto r = request(e.on_twist, state);
    if (!r.success) {
      logger_.warn() << r.message;
      entered_ = state.time;   // do not retry with the same command
    }
  }
  auto & active = modes_.at(current_);
  const auto status = active.mode->update(state, reference);
  if (status == ModeStatus::FAILED) {
    force_safe(state, "mode " + current_ + " failed");
    modes_.at(current_).mode->update(state, reference);
  } else if (status == ModeStatus::FINISHED && !active.next.empty() && !chain_failed_) {
    const auto r = request(active.next, state);
    if (!r.success) {
      chain_failed_ = true;
      logger_.warn() << "Cannot chain to " << active.next << ": " << r.message;
    } else {
      modes_.at(current_).mode->update(state, reference);
    }
  }
  return modes_.at(current_).controller;
}

}  // namespace gaitos
