//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_unitree_sim/UnitreeEmulatorBase.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>

namespace gaitos_unitree_sim
{

using namespace std::chrono_literals;

void UnitreeEmulatorBase::configure(
  rclcpp_lifecycle::LifecycleNode & node, gaitos::Gaitos & core, const gaitos::Params & p)
{
  node_ = &node;
  core_ = &core;
  api_prefix_ = p.get<std::string>("api_prefix", "/api");
  max_duration_ = p.get<double>("max_velocity_duration", 1.0);
  walking_modes_ = p.get<std::vector<std::string>>("walking_modes", {});
  // A request may wait for the control loop (mode change): its own group, so the state keeps
  // flowing meanwhile with a multi threaded executor
  rpc_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpc_subs_.clear();
  rpc_pubs_.clear();
  timers_.clear();
  timers_.push_back(node.create_wall_timer(20ms, [this]() {update_velocity();}));
  configure_robot(p);
}

void UnitreeEmulatorBase::deactivate()
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  velocity_active_ = false;
}

void UnitreeEmulatorBase::add_service(
  const std::string & name, const std::string & version, RpcHandler handler)
{
  auto pub = node_->create_publisher<unitree_api::msg::Response>(
    api_prefix_ + "/" + name + "/response", rclcpp::QoS(10).best_effort());
  rclcpp::SubscriptionOptions options;
  options.callback_group = rpc_group_;
  auto sub = node_->create_subscription<unitree_api::msg::Request>(
    api_prefix_ + "/" + name + "/request", rclcpp::QoS(10),
    [pub, version, handler](const unitree_api::msg::Request & req) {
      Reply r;
      const auto id = req.header.identity.api_id;
      if (id == api::kApiVersion) {
        r.data = version;
      } else if (id != api::kNoop) {
        r = handler(id, req.parameter);
      }
      unitree_api::msg::Response res;
      res.header.identity = req.header.identity;
      res.header.status.code = r.code;
      res.data = r.data;
      pub->publish(res);
    }, options);
  rpc_pubs_.push_back(pub);
  rpc_subs_.push_back(sub);
}

void UnitreeEmulatorBase::add_timer(double rate, std::function<void()> fn)
{
  timers_.push_back(node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate, 1e-3)), std::move(fn)));
}

std::vector<int> UnitreeEmulatorBase::motor_slots(const gaitos::Params & p, std::size_t slots) const
{
  const auto & model = *core_->model();
  std::vector<int> out(model.joint_count(), -1);
  const auto index = p.get_map("motor_index");
  if (index.empty()) {   // the model order is the motor order
    for (std::size_t i = 0; i < out.size() && i < slots; ++i) {
      out[i] = static_cast<int>(i);
    }
    return out;
  }
  for (const auto & [name, slot] : index) {
    const int k = static_cast<int>(std::lround(slot));
    if (k < 0 || k >= static_cast<int>(slots)) {
      throw std::runtime_error("unitree emulator: motor slot of " + name + " out of range");
    }
    bool found = false;
    for (std::size_t i = 0; i < model.joint_count(); ++i) {
      if (model.joint(i).name == name) {
        out[i] = k;
        found = true;
      }
    }
    if (!found) {
      RCLCPP_WARN(node_->get_logger(), "unitree emulator: joint %s of motor_index is not in the model",
        name.c_str());
    }
  }
  return out;
}

Reply UnitreeEmulatorBase::request_mode(const std::string & mode)
{
  Reply r;
  if (core_->snapshot().mode == mode) {
    return r;
  }
  auto f = core_->request_mode(mode);
  if (f.wait_for(1s) != std::future_status::ready) {
    r.code = code::kRejected;
    return r;
  }
  const auto result = f.get();
  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "unitree emulator: mode %s refused: %s", mode.c_str(),
      result.message.c_str());
    r.code = code::kRejected;
  }
  return r;
}

bool UnitreeEmulatorBase::walking() const
{
  const auto mode = core_->snapshot().mode;
  return std::find(walking_modes_.begin(), walking_modes_.end(), mode) != walking_modes_.end();
}

void UnitreeEmulatorBase::keep_velocity(const std::array<double, 3> & v, double duration)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  velocity_ = v;
  velocity_active_ = true;
  velocity_deadline_ = node_->now() +
    rclcpp::Duration::from_seconds(std::clamp(duration, 0.0, max_duration_));
  core_->set_twist({v[0], v[1], 0.0}, {0.0, 0.0, v[2]});
}

void UnitreeEmulatorBase::stop_velocity()
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  velocity_active_ = false;
  core_->set_twist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void UnitreeEmulatorBase::update_velocity()
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  if (!velocity_active_) {
    return;
  }
  if (node_->now() < velocity_deadline_) {
    // keep it fresh for the command timeout of the gaitos locomotion modes
    core_->set_twist({velocity_[0], velocity_[1], 0.0}, {0.0, 0.0, velocity_[2]});
  } else {
    core_->set_twist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    velocity_active_ = false;
  }
}

}  // namespace gaitos_unitree_sim
