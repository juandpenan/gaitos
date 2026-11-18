//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_naoqi_sim/NaoqiDriverEmulator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>

#include "gaitos_common/Math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gaitos_naoqi_sim
{

using namespace std::chrono_literals;

namespace
{
// NAOqi chains of the NAO H25 and the order of "Body" (ALMotion getBodyNames)
const std::vector<std::string> kChainOrder{"Head", "LArm", "LLeg", "RLeg", "RArm"};
const std::map<std::string, std::vector<std::string>> kChains{
  {"Head", {"HeadYaw", "HeadPitch"}},
  {"LArm", {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"}},
  {"LLeg", {"LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll"}},
  {"RLeg", {"RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"}},
  {"RArm", {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"}},
};

geometry_msgs::msg::Quaternion to_msg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion m;
  m.w = q.w();
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  return m;
}

Eigen::Quaterniond yaw_quaternion(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}
}  // namespace

void NaoqiDriverEmulator::configure(
  rclcpp_lifecycle::LifecycleNode & node, gaitos::Gaitos & core, const gaitos::Params & p)
{
  node_ = &node;
  core_ = &core;
  const auto & model = *core.model();
  odom_frame_ = p.get<std::string>("frames.odom", "odom");
  base_frame_ = p.get<std::string>("frames.base", "base_link");
  footprint_frame_ = p.get<std::string>("frames.footprint", "base_footprint");
  walking_modes_ = p.get<std::vector<std::string>>("walking_modes", {"balance_stand", "locomotion"});
  stand_modes_ = p.get<std::vector<std::string>>("stand_modes", {"stand_init", "stand_zero"});
  stand_mode_ = p.get<std::string>("stand_mode", "balance_stand");
  walk_mode_ = p.get<std::string>("walk_mode", "locomotion");
  default_speed_ = p.get<double>("default_speed", 0.1);

  // Chains: Head, LArm, LLeg, RLeg, RArm and Body (all of them in that order)
  chains_.clear();
  std::vector<std::string> body;
  for (const auto & name : kChainOrder) {
    chains_[name] = p.get<std::vector<std::string>>("chains." + name, kChains.at(name));
    body.insert(body.end(), chains_[name].begin(), chains_[name].end());
  }
  chains_["Body"] = body;

  // Joints of the body that the model computes from another one (RHipYawPitch)
  mimics_.clear();
  for (const auto & name : body) {
    if (model.joint_index(name)) {
      continue;
    }
    const auto it = model.tree().joints().find(name);
    if (it == model.tree().joints().end() || it->second.mimic.empty()) {
      RCLCPP_WARN(node.get_logger(), "naoqi emulator: joint %s of the body is not in the model",
        name.c_str());
      continue;
    }
    const auto master = model.joint_index(it->second.mimic);
    if (master) {
      mimics_.push_back({name, *master, it->second.mimic_multiplier, it->second.mimic_offset});
    }
  }

  const auto topic = [&p](const std::string & key, const std::string & def) {
      return p.get<std::string>("topics." + key, def);
    };
  joints_pub_ = node.create_publisher<sensor_msgs::msg::JointState>(
    topic("joint_states", "/joint_states"), 10);
  odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(topic("odom", "/odom"), 10);
  imu_pub_ = node.create_publisher<sensor_msgs::msg::Imu>(topic("imu", "/imu/torso"), 10);
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  // Commands may wait for a mode change of the control loop: their own group
  command_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = command_group_;
  cmd_vel_sub_ = node.create_subscription<geometry_msgs::msg::Twist>(
    topic("cmd_vel", "/cmd_vel"), 10, [this](const geometry_msgs::msg::Twist & m) {
      if (!move(m.linear.x, m.linear.y, m.angular.z)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "naoqi emulator: move ignored, the robot is not standing (mode %s)",
          core_->snapshot().mode.c_str());
      }
    }, options);
  angles_sub_ = node.create_subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>(
    topic("joint_angles", "/joint_angles"), 10,
    [this](const naoqi_bridge_msgs::msg::JointAnglesWithSpeed & m) {
      const auto targets = joint_targets(m);
      if (targets.empty() || core_->set_joint_targets(targets) == 0) {
        RCLCPP_WARN(node_->get_logger(), "naoqi emulator: joint_angles ignored (names / angles)");
      }
    }, options);
  goal_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
    topic("move_to", "/move_base_simple/goal"), 10,
    [this](const geometry_msgs::msg::PoseStamped & m) {on_goal(m);}, options);
  speech_sub_ = node.create_subscription<std_msgs::msg::String>(
    topic("speech", "/speech"), 10, [this](const std_msgs::msg::String & m) {
      RCLCPP_INFO(node_->get_logger(), "NAO says: %s", m.data.c_str());
    }, options);

  const auto every = [](double rate) {
      return std::chrono::duration<double>(1.0 / std::max(rate, 1e-3));
    };
  timers_.clear();
  timers_.push_back(node.create_wall_timer(every(p.get<double>("rates.joint_states", 50.0)),
    [this]() {publish_fast();}));
  timers_.push_back(node.create_wall_timer(every(p.get<double>("rates.odom", 15.0)),
    [this]() {publish_odom();}));
  timers_.push_back(node.create_wall_timer(every(p.get<double>("rates.imu", 10.0)),
    [this]() {publish_imu();}));
  timers_.push_back(node.create_wall_timer(20ms, [this]() {keep_velocity();}));

  RCLCPP_INFO(node.get_logger(), "naoqi emulator: %s, %s, %s, %s, %s", joints_pub_->get_topic_name(),
    odom_pub_->get_topic_name(), cmd_vel_sub_->get_topic_name(), angles_sub_->get_topic_name(),
    goal_sub_->get_topic_name());
}

void NaoqiDriverEmulator::deactivate()
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  moving_ = false;
}

bool NaoqiDriverEmulator::in(const std::vector<std::string> & modes, const std::string & mode) const
{
  return std::find(modes.begin(), modes.end(), mode) != modes.end();
}

bool NaoqiDriverEmulator::ensure_mode(const std::string & mode)
{
  if (core_->snapshot().mode == mode) {
    return true;
  }
  auto f = core_->request_mode(mode);
  if (f.wait_for(1s) != std::future_status::ready) {
    return false;
  }
  const auto r = f.get();
  if (!r.success && node_) {
    RCLCPP_WARN(node_->get_logger(), "naoqi emulator: mode %s refused: %s", mode.c_str(),
      r.message.c_str());
  }
  return r.success;
}

std::vector<gaitos::JointTarget> NaoqiDriverEmulator::joint_targets(
  const naoqi_bridge_msgs::msg::JointAnglesWithSpeed & msg) const
{
  std::vector<std::string> names = msg.joint_names;
  if (names.size() == 1 && chains_.count(names[0])) {   // a chain: Body, Head, LArm...
    names = chains_.at(names[0]);
  }
  std::vector<gaitos::JointTarget> out;
  if (names.size() != msg.joint_angles.size()) {
    return out;
  }
  const auto & model = *core_->model();
  const double fraction = msg.speed > 0.0 ? std::min<double>(msg.speed, 1.0) : default_speed_;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto idx = model.joint_index(names[i]);
    if (!idx) {   // RHipYawPitch follows LHipYawPitch, as on the robot
      continue;
    }
    const double vmax = model.joint(*idx).velocity > 0.0 ? model.joint(*idx).velocity : 1.0;
    out.push_back({names[i], msg.joint_angles[i], fraction * vmax, msg.relative != 0});
  }
  return out;
}

bool NaoqiDriverEmulator::move(double vx, double vy, double wz)
{
  const bool stop = std::abs(vx) + std::abs(vy) + std::abs(wz) < 1e-9;
  const auto mode = core_->snapshot().mode;
  if (stop) {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    moving_ = false;
    core_->set_twist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return true;
  }
  if (!in(walking_modes_, mode)) {
    if (!in(stand_modes_, mode) || !ensure_mode(stand_mode_)) {
      return false;
    }
  }
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  velocity_ = {vx, vy, wz};
  moving_ = true;
  core_->set_twist({vx, vy, 0.0}, {0.0, 0.0, wz});
  return true;
}

void NaoqiDriverEmulator::keep_velocity()
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  if (!moving_) {
    return;
  }
  if (!in(walking_modes_, core_->snapshot().mode)) {   // fell, rest... : the move is over
    moving_ = false;
    return;
  }
  core_->set_twist({velocity_.x(), velocity_.y(), 0.0}, {0.0, 0.0, velocity_.z()});
}

bool NaoqiDriverEmulator::move_to(double x, double y, double yaw)
{
  // The mode just requested, not the snapshot (it shows the change one cycle later)
  std::string mode = core_->snapshot().mode;
  if (mode != walk_mode_) {
    if (in(stand_modes_, mode)) {
      if (!ensure_mode(stand_mode_)) {
        return false;
      }
      mode = stand_mode_;
    }
    if (!in(walking_modes_, mode) || !ensure_mode(walk_mode_)) {
      return false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    moving_ = false;
  }
  core_->set_pose_goal({x, y, 0.0}, yaw_quaternion(yaw));
  return true;
}

void NaoqiDriverEmulator::on_goal(const geometry_msgs::msg::PoseStamped & m)
{
  const auto & o = m.pose.orientation;
  const bool has_q = std::abs(o.w) + std::abs(o.x) + std::abs(o.y) + std::abs(o.z) > 1e-9;
  double yaw = has_q ? gaitos::yaw_of(Eigen::Quaterniond(o.w, o.x, o.y, o.z)) : 0.0;
  Eigen::Vector2d p(m.pose.position.x, m.pose.position.y);
  if (m.header.frame_id == odom_frame_) {   // to the heading frame of the body
    const auto s = core_->snapshot();
    const double body_yaw = gaitos::yaw_of(s.base.orientation);
    const Eigen::Rotation2Dd r(-body_yaw);
    p = r * (p - s.base.position.head<2>());
    yaw = gaitos::wrap_angle(yaw - body_yaw);
  } else if (!m.header.frame_id.empty() && m.header.frame_id != footprint_frame_ &&
    m.header.frame_id != base_frame_)
  {
    RCLCPP_WARN(node_->get_logger(), "naoqi emulator: moveTo in frame '%s' ignored (use %s or %s)",
      m.header.frame_id.c_str(), odom_frame_.c_str(), footprint_frame_.c_str());
    return;
  }
  if (!move_to(p.x(), p.y(), yaw)) {
    RCLCPP_WARN(node_->get_logger(), "naoqi emulator: moveTo ignored, the robot is not standing");
  }
}

sensor_msgs::msg::JointState NaoqiDriverEmulator::joint_state(const gaitos::Snapshot & s) const
{
  const auto & model = *core_->model();
  sensor_msgs::msg::JointState js;
  for (const auto & name : chains_.at("Body")) {
    double q = 0.0;
    double dq = 0.0;
    double tau = 0.0;
    if (const auto idx = model.joint_index(name)) {
      q = *idx < s.joints.position.size() ? s.joints.position[*idx] : 0.0;
      dq = *idx < s.joints.velocity.size() ? s.joints.velocity[*idx] : 0.0;
      tau = *idx < s.joints.effort.size() ? s.joints.effort[*idx] : 0.0;
    } else {
      const auto m = std::find_if(mimics_.begin(), mimics_.end(),
          [&name](const Mimic & x) {return x.name == name;});
      if (m == mimics_.end() || m->master >= s.joints.position.size()) {
        continue;
      }
      q = m->multiplier * s.joints.position[m->master] + m->offset;
      dq = m->master < s.joints.velocity.size() ? m->multiplier * s.joints.velocity[m->master] : 0.0;
    }
    js.name.push_back(name);
    js.position.push_back(q);
    js.velocity.push_back(dq);
    js.effort.push_back(tau);
  }
  return js;
}

void NaoqiDriverEmulator::publish_fast()
{
  const auto s = core_->snapshot();
  if (s.cycle == 0) {
    return;
  }
  const auto t = stamp();
  auto js = joint_state(s);
  js.header.stamp = t;
  joints_pub_->publish(js);
  if (!s.base.valid) {
    return;
  }
  // odom -> base_link, and base_footprint: on the ground under the torso, yaw only
  geometry_msgs::msg::TransformStamped odom;
  odom.header.stamp = t;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.transform.translation.x = s.base.position.x();
  odom.transform.translation.y = s.base.position.y();
  odom.transform.translation.z = s.base.position.z();
  odom.transform.rotation = to_msg(s.base.orientation);

  const auto & model = *core_->model();
  double ground = std::numeric_limits<double>::infinity();
  if (s.joints.position.size() == model.joint_count()) {
    for (const auto & leg : model.legs()) {
      const Eigen::Vector3d sole = s.base.position +
        s.base.orientation * leg.chain.forward(s.joints.position).translation();
      ground = std::min(ground, sole.z());
    }
  }
  if (!std::isfinite(ground)) {
    ground = 0.0;
  }
  const Eigen::Isometry3d base = Eigen::Translation3d(s.base.position) * s.base.orientation;
  const Eigen::Isometry3d footprint =
    Eigen::Translation3d(s.base.position.x(), s.base.position.y(), ground) *
    yaw_quaternion(gaitos::yaw_of(s.base.orientation));
  const Eigen::Isometry3d rel = base.inverse() * footprint;
  geometry_msgs::msg::TransformStamped fp;
  fp.header.stamp = t;
  fp.header.frame_id = base_frame_;
  fp.child_frame_id = footprint_frame_;
  fp.transform.translation.x = rel.translation().x();
  fp.transform.translation.y = rel.translation().y();
  fp.transform.translation.z = rel.translation().z();
  fp.transform.rotation = to_msg(Eigen::Quaterniond(rel.rotation()));
  tf_->sendTransform({odom, fp});
}

void NaoqiDriverEmulator::publish_odom()
{
  const auto s = core_->snapshot();
  if (s.cycle == 0 || !s.base.valid) {
    return;
  }
  nav_msgs::msg::Odometry m;
  m.header.stamp = stamp();
  m.header.frame_id = odom_frame_;
  m.child_frame_id = base_frame_;
  m.pose.pose.position.x = s.base.position.x();
  m.pose.pose.position.y = s.base.position.y();
  m.pose.pose.position.z = s.base.position.z();
  m.pose.pose.orientation = to_msg(s.base.orientation);
  const Eigen::Vector3d v = s.base.orientation.inverse() * s.base.linear_velocity;   // child frame
  m.twist.twist.linear.x = v.x();
  m.twist.twist.linear.y = v.y();
  m.twist.twist.linear.z = v.z();
  m.twist.twist.angular.x = s.base.angular_velocity.x();
  m.twist.twist.angular.y = s.base.angular_velocity.y();
  m.twist.twist.angular.z = s.base.angular_velocity.z();
  odom_pub_->publish(m);
}

void NaoqiDriverEmulator::publish_imu()
{
  const auto s = core_->snapshot();
  if (s.cycle == 0) {
    return;
  }
  sensor_msgs::msg::Imu m;
  m.header.stamp = stamp();
  m.header.frame_id = base_frame_;
  m.orientation = to_msg(s.imu.has_orientation ? s.imu.orientation : s.base.orientation);
  m.angular_velocity.x = s.imu.angular_velocity.x();
  m.angular_velocity.y = s.imu.angular_velocity.y();
  m.angular_velocity.z = s.imu.angular_velocity.z();
  m.linear_acceleration.x = s.imu.linear_acceleration.x();
  m.linear_acceleration.y = s.imu.linear_acceleration.y();
  m.linear_acceleration.z = s.imu.linear_acceleration.z();
  imu_pub_->publish(m);
}

}  // namespace gaitos_naoqi_sim

PLUGINLIB_EXPORT_CLASS(gaitos_naoqi_sim::NaoqiDriverEmulator, gaitos::NodeExtension)
