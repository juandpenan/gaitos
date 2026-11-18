//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_unitree_sim/UnitreeGoEmulator.hpp"

#include <algorithm>
#include <cmath>

#include "nlohmann/json.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_unitree_sim
{

namespace
{
constexpr std::size_t kMotors = 20;   // LowState.motor_state of unitree_go
constexpr std::size_t kFeet = 4;

int16_t clamp16(double v)
{
  return static_cast<int16_t>(std::clamp(std::lround(v), -32768L, 32767L));
}
}  // namespace

void UnitreeGoEmulator::configure_robot(const gaitos::Params & p)
{
  if (walking_modes_.empty()) {
    walking_modes_ = {"balance_stand", "locomotion", "locomotion_mpc"};
  }
  pose_modes_ = p.get<std::vector<std::string>>("pose_modes", {"balance_stand"});
  move_timeout_ = p.get<double>("move_timeout", 1.0);
  motor_of_joint_ = motor_slots(p, kMotors);

  // Feet in the Unitree order
  const auto & model = *core().model();
  const auto order = p.get<std::vector<std::string>>("leg_order", {"FR", "FL", "RR", "RL"});
  leg_of_slot_.assign(kFeet, -1);
  for (std::size_t k = 0; k < order.size() && k < kFeet; ++k) {
    for (std::size_t l = 0; l < model.leg_count(); ++l) {
      if (model.leg(l).name == order[k]) {
        leg_of_slot_[k] = static_cast<int>(l);
      }
    }
  }

  // Sport api -> gaitos mode, gaitos mode -> sport mode reported
  const auto ids = p.get<std::vector<double>>("api.ids", {1001, 1002, 1004, 1005, 1006, 1009, 1010});
  const auto modes = p.get<std::vector<std::string>>("api.modes",
      {"damp", "balance_stand", "stand_up", "stand_down", "recovery_stand", "sit", "balance_stand"});
  if (ids.size() != modes.size()) {
    throw std::runtime_error("unitree emulator: api.ids and api.modes differ in size");
  }
  api_modes_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i) {
    api_modes_[std::llround(ids[i])] = modes[i];
  }
  // Sport modes of SportModeState: 1 balance stand, 3 locomotion, 5 lie down, 6 joint lock,
  // 7 damping, 8 recovery stand, 10 sit
  const auto report_modes = p.get<std::vector<std::string>>("sport_modes.modes",
      {"damp", "zero_torque", "stand_up", "stand_down", "recovery_stand", "sit", "balance_stand",
        "locomotion", "locomotion_mpc"});
  const auto report_codes = p.get<std::vector<double>>("sport_modes.codes",
      {7, 7, 6, 5, 8, 10, 1, 3, 3});
  if (report_modes.size() != report_codes.size()) {
    throw std::runtime_error("unitree emulator: sport_modes.modes and .codes differ in size");
  }
  sport_modes_.clear();
  for (std::size_t i = 0; i < report_modes.size(); ++i) {
    sport_modes_[report_modes[i]] = static_cast<int>(std::lround(report_codes[i]));
  }

  const auto topic = [&p](const std::string & key, const std::string & def) {
      return p.get<std::string>("topics." + key, def);
    };
  auto & n = node();
  lowstate_pub_ = n.create_publisher<unitree_go::msg::LowState>(topic("lowstate", "/lowstate"),
      state_qos());
  lf_lowstate_pub_ = n.create_publisher<unitree_go::msg::LowState>(
    topic("lf_lowstate", "/lf/lowstate"), state_qos());
  sport_pub_ = n.create_publisher<unitree_go::msg::SportModeState>(
    topic("sportmodestate", "/sportmodestate"), state_qos());
  lf_sport_pub_ = n.create_publisher<unitree_go::msg::SportModeState>(
    topic("lf_sportmodestate", "/lf/sportmodestate"), state_qos());

  add_service("sport", p.get<std::string>("versions.sport", "1.0.0.1"),
    [this](int64_t id, const std::string & prm) {return handle_sport(id, prm);});

  const auto low = [this](rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr pub) {
      return [this, pub]() {
               const auto s = core().snapshot();
               if (s.cycle > 0) {
                 unitree_go::msg::LowState msg;
                 fill_lowstate(s, msg);
                 pub->publish(msg);
               }
             };
    };
  const auto sport = [this](rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr pub) {
      return [this, pub]() {
               const auto s = core().snapshot();
               unitree_go::msg::SportModeState msg;
               if (s.cycle > 0 && fill_sport_state(s, msg)) {
                 pub->publish(msg);
               }
             };
    };
  add_timer(p.get<double>("rates.lowstate", 500.0), low(lowstate_pub_));
  add_timer(p.get<double>("rates.lf_lowstate", 20.0), low(lf_lowstate_pub_));
  add_timer(p.get<double>("rates.sportmodestate", 200.0), sport(sport_pub_));
  add_timer(p.get<double>("rates.lf_sportmodestate", 20.0), sport(lf_sport_pub_));

  RCLCPP_INFO(n.get_logger(), "unitree emulator (go): %s, %s, %s/sport",
    lowstate_pub_->get_topic_name(), sport_pub_->get_topic_name(), api_prefix_.c_str());
}

std::optional<int> UnitreeGoEmulator::sport_mode_of(const std::string & mode) const
{
  const auto it = sport_modes_.find(mode);
  if (it == sport_modes_.end()) {
    return std::nullopt;
  }
  return it->second;
}

Reply UnitreeGoEmulator::handle_sport(int64_t api_id, const std::string & parameter)
{
  Reply r;
  const auto it = api_modes_.find(api_id);
  if (it != api_modes_.end()) {
    stop_velocity();
    return request_mode(it->second);
  }
  // {"x", "y", "z"} of Euler and Move
  const auto xyz = [&parameter](std::array<double, 3> & v) {
      try {
        const auto js = nlohmann::json::parse(parameter);
        v = {js.at("x").get<double>(), js.at("y").get<double>(), js.at("z").get<double>()};
        return true;
      } catch (const nlohmann::json::exception &) {
        return false;
      }
    };
  switch (api_id) {
    case go2::kMove: {
        std::array<double, 3> v{};
        if (!xyz(v)) {
          r.code = code::kBadParameter;
          return r;
        }
        if (!walking()) {
          r.code = code::kRejected;
          return r;
        }
        keep_velocity(v, move_timeout_);
        return r;
      }
    case go2::kStopMove:
      stop_velocity();
      return r;
    case go2::kEuler: {
        std::array<double, 3> rpy{};
        if (!xyz(rpy)) {
          r.code = code::kBadParameter;
          return r;
        }
        const auto mode = core().snapshot().mode;
        if (std::find(pose_modes_.begin(), pose_modes_.end(), mode) == pose_modes_.end()) {
          r.code = code::kRejected;
          return r;
        }
        const Eigen::Quaterniond q =
          Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
        core().set_pose_goal(Eigen::Vector3d::Zero(), q);
        return r;
      }
    case go2::kSpeedLevel:
    case go2::kSwitchJoystick:
    case go2::kPose:
      return r;
    default:
      r.code = code::kNotImplemented;
      return r;
  }
}

void UnitreeGoEmulator::fill_lowstate(const gaitos::Snapshot & s, unitree_go::msg::LowState & msg) const
{
  msg.head = {0xFE, 0xEF};
  msg.level_flag = 0xFF;
  msg.tick = static_cast<uint32_t>(std::fmod(s.time * 1000.0, 4294967296.0));
  fill_imu(s.imu, s.base, msg.imu_state);
  for (std::size_t i = 0; i < motor_of_joint_.size() && i < s.joints.position.size(); ++i) {
    const int k = motor_of_joint_[i];
    if (k < 0) {
      continue;
    }
    auto & m = msg.motor_state[static_cast<std::size_t>(k)];
    m.mode = 1;
    m.q = m.q_raw = static_cast<float>(s.joints.position[i]);
    m.dq = m.dq_raw = i < s.joints.velocity.size() ? static_cast<float>(s.joints.velocity[i]) : 0.0f;
    m.tau_est = i < s.joints.effort.size() ? static_cast<float>(s.joints.effort[i]) : 0.0f;
    m.temperature = 35;
  }
  for (std::size_t k = 0; k < kFeet; ++k) {
    const int l = leg_of_slot_[k];
    if (l >= 0 && static_cast<std::size_t>(l) < s.contacts.force.size()) {
      msg.foot_force[k] = msg.foot_force_est[k] = clamp16(s.contacts.force[static_cast<std::size_t>(l)]);
    }
  }
  msg.bms_state.soc = 100;   // a full, idle battery
  msg.power_v = 28.0f;
  msg.crc = 0;
}

bool UnitreeGoEmulator::fill_sport_state(
  const gaitos::Snapshot & s, unitree_go::msg::SportModeState & msg) const
{
  if (!s.base.valid) {
    return false;
  }
  const auto & model = *core().model();
  msg.stamp.sec = static_cast<int32_t>(std::floor(s.time));
  msg.stamp.nanosec = static_cast<uint32_t>((s.time - std::floor(s.time)) * 1e9);
  gaitos::ImuData imu = s.imu;
  imu.orientation = s.base.orientation;
  imu.has_orientation = true;
  fill_imu(imu, s.base, msg.imu_state);
  const auto mode = sport_mode_of(s.mode);
  msg.mode = static_cast<uint8_t>(mode.value_or(0));
  msg.gait_type = static_cast<uint8_t>(mode && *mode == 3 ? 1 : 0);   // trot while walking
  msg.foot_raise_height = 0.08f;
  for (int i = 0; i < 3; ++i) {
    msg.position[static_cast<std::size_t>(i)] = static_cast<float>(s.base.position(i));
    msg.velocity[static_cast<std::size_t>(i)] = static_cast<float>(s.base.linear_velocity(i));
  }
  msg.body_height = static_cast<float>(s.base.position.z());
  msg.yaw_speed = static_cast<float>((s.base.orientation * s.base.angular_velocity).z());
  for (std::size_t k = 0; k < kFeet; ++k) {
    const int l = leg_of_slot_[k];
    if (l < 0 || s.joints.position.size() != model.joint_count()) {
      continue;
    }
    const auto & chain = model.leg(static_cast<std::size_t>(l)).chain;
    const Eigen::Vector3d p = chain.forward(s.joints.position).translation();
    Eigen::VectorXd dq(static_cast<Eigen::Index>(chain.dof()));
    for (std::size_t j = 0; j < chain.dof(); ++j) {
      const auto idx = static_cast<std::size_t>(chain.joint_indices()[j]);
      dq(static_cast<Eigen::Index>(j)) = idx < s.joints.velocity.size() ? s.joints.velocity[idx] : 0.0;
    }
    const Eigen::Vector3d v = chain.jacobian(s.joints.position).topRows(3) * dq;
    for (int a = 0; a < 3; ++a) {
      msg.foot_position_body[3 * k + static_cast<std::size_t>(a)] = static_cast<float>(p(a));
      msg.foot_speed_body[3 * k + static_cast<std::size_t>(a)] = static_cast<float>(v(a));
    }
    if (static_cast<std::size_t>(l) < s.contacts.force.size()) {
      msg.foot_force[k] = clamp16(s.contacts.force[static_cast<std::size_t>(l)]);
    }
  }
  return true;
}

}  // namespace gaitos_unitree_sim

PLUGINLIB_EXPORT_CLASS(gaitos_unitree_sim::UnitreeGoEmulator, gaitos::NodeExtension)
