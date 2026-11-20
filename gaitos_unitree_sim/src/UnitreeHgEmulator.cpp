//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_unitree_sim/UnitreeHgEmulator.hpp"

#include <algorithm>
#include <cmath>

#include "nlohmann/json.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_unitree_sim
{

namespace
{
constexpr std::size_t kMotors = 35;   // LowState.motor_state of unitree_hg
}

void UnitreeHgEmulator::configure_robot(const gaitos::Params & p)
{
  if (walking_modes_.empty()) {
    walking_modes_ = {"balance_stand", "locomotion_lipm", "locomotion"};
  }
  motor_of_joint_ = motor_slots(p, kMotors);
  mode_machine_ = static_cast<uint8_t>(p.get<int>("mode_machine", 0));

  // FSM of the locomotion service <-> gaitos modes
  const auto ids = p.get<std::vector<double>>("fsm.ids", {0, 1, 4, 811});
  const auto modes = p.get<std::vector<std::string>>("fsm.modes",
      {"zero_torque", "damp", "stand_up", "balance_stand"});
  if (ids.size() != modes.size()) {
    throw std::runtime_error("unitree emulator: fsm.ids and fsm.modes differ in size");
  }
  fsm_requests_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i) {
    fsm_requests_.emplace_back(static_cast<int>(std::lround(ids[i])), modes[i]);
  }
  const auto report_modes = p.get<std::vector<std::string>>("fsm.report_modes",
      {"zero_torque", "damp", "stand_up", "balance_stand", "locomotion_lipm", "locomotion"});
  const auto report_ids = p.get<std::vector<double>>("fsm.report_ids", {0, 1, 4, 811, 811, 811});
  if (report_modes.size() != report_ids.size()) {
    throw std::runtime_error("unitree emulator: fsm.report_modes and fsm.report_ids differ in size");
  }
  fsm_reports_.clear();
  for (std::size_t i = 0; i < report_modes.size(); ++i) {
    fsm_reports_[report_modes[i]] = static_cast<int>(std::lround(report_ids[i]));
  }

  const auto topic = [&p](const std::string & key, const std::string & def) {
      return p.get<std::string>("topics." + key, def);
    };
  auto & n = node();
  lowstate_pub_ = n.create_publisher<unitree_hg::msg::LowState>(topic("lowstate", "/lowstate"),
      state_qos());
  imu_pub_ = n.create_publisher<unitree_hg::msg::IMUState>(
    topic("secondary_imu", "/secondary_imu"), state_qos());
  bms_pub_ = n.create_publisher<unitree_hg::msg::BmsState>(topic("bms", "/lf/bmsstate"), state_qos());
  odom_pub_ = n.create_publisher<unitree_go::msg::SportModeState>(
    topic("odometry", "/odommodestate"), state_qos());
  arm_state_pub_ = n.create_publisher<std_msgs::msg::String>(
    topic("arm_state", "/arm/action/state"), state_qos());

  // arm_sdk: upper body commands over the walking controller
  const gaitos::Params arm = p.sub("arm_sdk");
  arm_sdk_joints_.clear();
  if (arm.get<bool>("enabled", false)) {
    const auto & model = *core().model();
    for (const auto & name : arm.get<std::vector<std::string>>("joints", {})) {
      const auto idx = model.joint_index(name);
      if (!idx) {
        throw std::runtime_error("unitree emulator: arm_sdk joint '" + name + "' is not in the model");
      }
      if (motor_of_joint_[*idx] < 0) {
        throw std::runtime_error("unitree emulator: arm_sdk joint '" + name + "' has no motor slot");
      }
      arm_sdk_joints_.push_back(*idx);
    }
    if (arm_sdk_joints_.empty()) {   // default: the joints of the SRDF group arm_sdk
      for (const auto i : model.group_indices("arm_sdk")) {
        if (motor_of_joint_[i] >= 0) {
          arm_sdk_joints_.push_back(i);
        }
      }
    }
    for (const auto i : arm_sdk_joints_) {   // gaitos would drop their commands
      if (!core().external_joint_allowed(i, walking_modes_.front())) {
        RCLCPP_WARN(n.get_logger(), "unitree emulator: arm_sdk joint '%s' is not in "
          "external_joints.groups: its commands are ignored (see params/r1_arm_sdk.yaml)",
          model.joint(i).name.c_str());
      }
    }
    arm_sdk_timeout_ = arm.get<double>("timeout", 0.5);
    if (arm_sdk_timeout_ <= 0.0) {
      throw std::runtime_error("unitree emulator: arm_sdk.timeout must be > 0");
    }
    held_fsm_ = arm.get<int>("held_fsm", 816);
    held_velocity_code_ = arm.get<int>("velocity_code", 127);
    held_fsm_code_ = arm.get<int>("fsm_code", 1001);
    // The robot's reader is best effort, depth 1 (unitree_r1 and the SDK publish so)
    arm_sdk_sub_ = n.create_subscription<unitree_hg::msg::LowCmd>(
      topic("arm_sdk", "/arm_sdk"), rclcpp::QoS(1).best_effort(),
      [this](const unitree_hg::msg::LowCmd & msg) {
        const auto commands = arm_sdk_commands(msg);
        if (!commands.empty()) {
          core().set_joint_commands(commands);
        }
      });
  }

  add_service("sport", p.get<std::string>("versions.sport", "1.0.0.1"),
    [this](int64_t id, const std::string & prm) {return handle_sport(id, prm);});
  add_service("arm", p.get<std::string>("versions.arm", "1.0.0.1"),
    [this](int64_t id, const std::string & prm) {return handle_arm(id, prm);});

  add_timer(p.get<double>("rates.lowstate", 500.0), [this]() {
      const auto s = core().snapshot();
      if (s.cycle > 0) {
        unitree_hg::msg::LowState msg;
        fill_lowstate(s, msg);
        lowstate_pub_->publish(msg);
      }
    });
  add_timer(p.get<double>("rates.odometry", 100.0), [this]() {
      const auto s = core().snapshot();
      unitree_go::msg::SportModeState msg;
      if (s.cycle > 0 && fill_odometry(s, msg)) {
        odom_pub_->publish(msg);
      }
    });
  add_timer(p.get<double>("rates.slow", 10.0), [this]() {
      const auto s = core().snapshot();
      if (s.cycle == 0) {
        return;
      }
      unitree_hg::msg::IMUState imu;
      fill_imu(s.imu, s.base, imu);
      imu_pub_->publish(imu);
      unitree_hg::msg::BmsState bms;   // a full, idle battery
      bms.soc = 100;
      bms.soh = 100;
      bms.bmsvoltage[0] = 50000;   // mV
      std::fill(bms.temperature.begin(), bms.temperature.end(), 25);
      bms_pub_->publish(bms);
      std_msgs::msg::String arm;
      arm.data = R"({"holding": false, "id": -1, "name": ""})";
      arm_state_pub_->publish(arm);
    });

  RCLCPP_INFO(n.get_logger(), "unitree emulator (hg): %s, %s, %s/sport and %s/arm",
    lowstate_pub_->get_topic_name(), odom_pub_->get_topic_name(), api_prefix_.c_str(),
    api_prefix_.c_str());
  if (arm_sdk_sub_) {
    RCLCPP_INFO(n.get_logger(), "unitree emulator (hg): arm_sdk on %s, %zu joints",
      arm_sdk_sub_->get_topic_name(), arm_sdk_joints_.size());
  }
}

bool UnitreeHgEmulator::held() const
{
  const double last = last_held_;
  return last >= 0.0 && walking() &&
         node().now().seconds() - last <= arm_sdk_timeout_;
}

std::vector<gaitos::JointCommandOverride> UnitreeHgEmulator::arm_sdk_commands(
  const unitree_hg::msg::LowCmd & msg)
{
  std::vector<gaitos::JointCommandOverride> out;
  // The robot ignores arm_sdk outside the walking FSM (811)
  if (arm_sdk_joints_.empty() || !walking()) {
    last_held_ = -1.0;
    return out;
  }
  const double weight = std::clamp(static_cast<double>(msg.mode_pr) / 100.0, 0.0, 1.0);
  last_held_ = weight > 0.0 ? node().now().seconds() : -1.0;
  const auto & model = *core().model();
  for (const auto i : arm_sdk_joints_) {
    const auto & m = msg.motor_cmd[static_cast<std::size_t>(motor_of_joint_[i])];
    gaitos::JointCommandOverride c;
    c.name = model.joint(i).name;
    c.position = m.q;
    c.velocity = m.dq;
    c.effort = m.tau;
    c.kp = m.kp;
    c.kd = m.kd;
    c.weight = weight;
    out.push_back(c);
  }
  return out;
}

std::optional<int> UnitreeHgEmulator::fsm_of_mode(const std::string & mode) const
{
  const auto it = fsm_reports_.find(mode);
  if (it == fsm_reports_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<std::string> UnitreeHgEmulator::mode_of_fsm(int fsm) const
{
  for (const auto & [id, mode] : fsm_requests_) {
    if (id == fsm) {
      return mode;
    }
  }
  return std::nullopt;
}

Reply UnitreeHgEmulator::handle_sport(int64_t api_id, const std::string & parameter)
{
  Reply r;
  switch (api_id) {
    case hg::kGetFsmId: {
        auto fsm = fsm_of_mode(core().snapshot().mode);
        if (!fsm) {
          r.code = code::kRejected;   // a gaitos mode with no FSM
          return r;
        }
        if (held()) {
          fsm = held_fsm_;   // the R1 switches to 816 while arm_sdk holds the arms
        }
        nlohmann::json js;
        js["data"] = *fsm;
        r.data = js.dump();
        return r;
      }
    case hg::kSetFsmId: {
        int fsm = 0;
        try {
          fsm = nlohmann::json::parse(parameter).at("data").get<int>();
        } catch (const nlohmann::json::exception &) {
          r.code = code::kBadParameter;
          return r;
        }
        const auto mode = mode_of_fsm(fsm);
        if (!mode) {
          r.code = code::kBadParameter;
          return r;
        }
        if (held() && fsm == 811) {
          r.code = held_fsm_code_;   // no walking FSM while the arms are held
          return r;
        }
        return request_mode(*mode);
      }
    case hg::kSetVelocity: {
        std::array<double, 3> v{};
        double duration = 0.0;
        try {
          const auto js = nlohmann::json::parse(parameter);
          const auto vel = js.at("velocity");
          for (std::size_t i = 0; i < 3; ++i) {
            v[i] = vel.at(i).get<double>();
          }
          duration = js.value("duration", 1.0);
        } catch (const nlohmann::json::exception &) {
          r.code = code::kBadParameter;
          return r;
        }
        if (!walking()) {
          r.code = code::kRejected;   // the robot only walks in FSM 811
          return r;
        }
        if (held()) {
          r.code = held_velocity_code_;   // nor while arm_sdk holds the arms (FSM 816)
          return r;
        }
        keep_velocity(v, duration);
        return r;
      }
    case hg::kSetSpeedMode:
      return r;
    default:
      r.code = code::kNotImplemented;
      return r;
  }
}

Reply UnitreeHgEmulator::handle_arm(int64_t api_id, const std::string &)
{
  Reply r;
  switch (api_id) {
    case hg::kGetActionList:
      r.data = "[[],[]]";   // no preset, no custom action
      return r;
    case hg::kStopCustomAction:
      return r;
    default:   // the actions themselves are not simulated
      r.code = code::kNotImplemented;
      return r;
  }
}

void UnitreeHgEmulator::fill_lowstate(const gaitos::Snapshot & s, unitree_hg::msg::LowState & msg) const
{
  msg.mode_pr = 0;
  msg.mode_machine = mode_machine_;
  msg.tick = static_cast<uint32_t>(std::fmod(s.time * 1000.0, 4294967296.0));
  fill_imu(s.imu, s.base, msg.imu_state);
  for (std::size_t i = 0; i < motor_of_joint_.size() && i < s.joints.position.size(); ++i) {
    const int k = motor_of_joint_[i];
    if (k < 0) {
      continue;
    }
    auto & m = msg.motor_state[static_cast<std::size_t>(k)];
    m.mode = 1;
    m.q = static_cast<float>(s.joints.position[i]);
    m.dq = i < s.joints.velocity.size() ? static_cast<float>(s.joints.velocity[i]) : 0.0f;
    m.tau_est = i < s.joints.effort.size() ? static_cast<float>(s.joints.effort[i]) : 0.0f;
    m.temperature = {40, 40};
    m.vol = 48.0f;
  }
  msg.crc = 0;
}

bool UnitreeHgEmulator::fill_odometry(
  const gaitos::Snapshot & s, unitree_go::msg::SportModeState & msg) const
{
  if (!s.base.valid) {
    return false;
  }
  msg.stamp.sec = static_cast<int32_t>(std::floor(s.time));
  msg.stamp.nanosec = static_cast<uint32_t>((s.time - std::floor(s.time)) * 1e9);
  gaitos::ImuData imu = s.imu;
  imu.orientation = s.base.orientation;
  imu.has_orientation = true;
  fill_imu(imu, s.base, msg.imu_state);
  const auto fsm = fsm_of_mode(s.mode);
  msg.mode = static_cast<uint8_t>(fsm && *fsm == 811 ? 1 : 0);
  for (int i = 0; i < 3; ++i) {
    msg.position[static_cast<std::size_t>(i)] = static_cast<float>(s.base.position(i));
    msg.velocity[static_cast<std::size_t>(i)] = static_cast<float>(s.base.linear_velocity(i));
  }
  msg.body_height = static_cast<float>(s.base.position.z());
  msg.yaw_speed = static_cast<float>((s.base.orientation * s.base.angular_velocity).z());
  return true;
}

}  // namespace gaitos_unitree_sim

PLUGINLIB_EXPORT_CLASS(gaitos_unitree_sim::UnitreeHgEmulator, gaitos::NodeExtension)
