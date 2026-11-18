//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_system/GaitosNode.hpp"

#include <chrono>
#include <cmath>

#include "gaitos_common/Math.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace gaitos
{

using namespace std::chrono_literals;

GaitosNode::GaitosNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(
    "gaitos",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
}

GaitosNode::~GaitosNode()
{
  running_ = false;
  if (loop_.joinable()) {
    loop_.join();
  }
}

GaitosNode::CallbackReturn GaitosNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    core_.configure(Params(get_node_parameters_interface()), hardware_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration error: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  const Params p(get_node_parameters_interface());
  odom_frame_ = p.get<std::string>("frames.odom", "odom");
  base_frame_ = p.get<std::string>("frames.base", core_.model()->base_link());
  const auto prefix = p.get<std::string>("frames.prefix", "");
  odom_frame_ = prefix + odom_frame_;
  base_frame_ = prefix + base_frame_;
  publish_tf_ = p.get<bool>("publish_tf", true);
  publish_joints_ = p.get<bool>("publish.joint_states", true);
  publish_odometry_ = p.get<bool>("publish.odometry", true);
  publish_clock_ = p.get<bool>("publish_clock", false);
  create_interfaces();
  if (!load_extensions()) {
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Configured %s: modes on %s/mode/<name> and %s/set_mode",
    core_.model()->name().c_str(), get_namespace(), get_namespace());
  return CallbackReturn::SUCCESS;
}

void GaitosNode::create_interfaces()
{
  const Params p(get_node_parameters_interface());
  twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::Twist & m) {
      core_.set_twist({m.linear.x, m.linear.y, m.linear.z}, {m.angular.x, m.angular.y, m.angular.z});
    });
  targets_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_targets", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::JointState & m) {
      std::vector<JointTarget> targets;
      for (std::size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
        targets.push_back({m.name[i], m.position[i], i < m.velocity.size() ? m.velocity[i] : 0.0, false});
      }
      core_.set_joint_targets(targets);
    });
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped & m) {on_goal(m);});

  // External odometries: sources[i] from topics[i]
  const auto sources = p.get<std::vector<std::string>>("external_odometry.sources", {});
  const auto topics = p.get<std::vector<std::string>>("external_odometry.topics", {});
  for (std::size_t i = 0; i < sources.size() && i < topics.size(); ++i) {
    const auto source = sources[i];
    odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        topics[i], rclcpp::SensorDataQoS(), [this, source](const nav_msgs::msg::Odometry & m) {
          Odometry o;
          o.stamp = rclcpp::Time(m.header.stamp).seconds();
          o.position = {m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z};
          const auto & q = m.pose.pose.orientation;
          o.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
          o.linear_velocity = {m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z};
          o.angular_velocity = {m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z};
          for (int k = 0; k < 3; ++k) {
            o.position_variance(k) = m.pose.covariance[static_cast<std::size_t>(7 * k)];
            o.velocity_variance(k) = m.twist.covariance[static_cast<std::size_t>(7 * k)];
          }
          o.yaw_variance = m.pose.covariance[35];
          core_.push_odometry(source, o);
        }));
    RCLCPP_INFO(get_logger(), "External odometry '%s' from %s", source.c_str(), topics[i].c_str());
  }

  // Joystick (sensor_msgs/Joy) as the hand held remote
  if (p.get<bool>("teleop.joy.enabled", false)) {
    // Joy button index of R1, L1, START, SELECT, R2, L2, F1, F2, A, B, X, Y, UP, RIGHT, DOWN,
    // LEFT (-1: none); axes: left x, left y, right x, right y (negative index: inverted)
    const auto b = p.get<std::vector<double>>("teleop.joy.buttons",
        {5, 4, 7, 6, -1, -1, -1, -1, 0, 1, 2, 3, -1, -1, -1, -1});
    const auto a = p.get<std::vector<double>>("teleop.joy.axes", {-1, 2, -4, 5});
    joy_buttons_.assign(b.begin(), b.end());
    joy_axes_.assign(a.begin(), a.end());
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      p.get<std::string>("teleop.joy.topic", "joy"), rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Joy & m) {on_joy(m);});
  }

  // Modes
  set_mode_srv_ = create_service<gaitos_msgs::srv::SetMode>(
    "set_mode", [this](const std::shared_ptr<gaitos_msgs::srv::SetMode::Request> req,
    std::shared_ptr<gaitos_msgs::srv::SetMode::Response> res) {
      const auto r = request(req->mode);
      res->success = r.success;
      res->message = r.message;
    });
  for (const auto & mode : core_.modes()) {
    mode_srvs_.push_back(create_service<std_srvs::srv::Trigger>(
        "mode/" + mode, [this, mode](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          const auto r = request(mode);
          res->success = r.success;
          res->message = r.message;
        }));
  }

  // State
  mode_pub_ = create_publisher<std_msgs::msg::String>("mode", rclcpp::QoS(1).transient_local());
  status_pub_ = create_publisher<gaitos_msgs::msg::Status>("status", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
  truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);
  joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  if (publish_clock_) {
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  }
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  const double rate = p.get<double>("publish_rate", 50.0);
  publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {publish();});
}

bool GaitosNode::load_extensions()
{
  const Params block = Params(get_node_parameters_interface()).sub("extensions");
  const auto names = block.keys();
  if (names.empty()) {
    return true;
  }
  try {
    if (!extension_loader_) {
      extension_loader_ = std::make_unique<pluginlib::ClassLoader<NodeExtension>>(
        "gaitos_system", "gaitos::NodeExtension");
    }
    for (const auto & name : names) {
      const Params params = block.sub(name);
      const auto type = params.require<std::string>("plugin");
      auto ext = extension_loader_->createSharedInstance(type);
      ext->configure(*this, core_, params);
      extensions_.push_back(ext);
      RCLCPP_INFO(get_logger(), "Extension '%s': %s", name.c_str(), type.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Extension error: %s", e.what());
    extensions_.clear();
    return false;
  }
  return true;
}

CommandResult GaitosNode::request(const std::string & mode)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return CommandResult::fail("the node is not active");
  }
  auto f = core_.request_mode(mode);
  if (f.wait_for(2s) != std::future_status::ready) {
    return CommandResult::fail("no answer from the control loop");
  }
  return f.get();
}

void GaitosNode::on_goal(const geometry_msgs::msg::PoseStamped & m)
{
  Eigen::Vector3d p(m.pose.position.x, m.pose.position.y, m.pose.position.z);
  const auto & o = m.pose.orientation;
  const bool has_q = std::abs(o.w) + std::abs(o.x) + std::abs(o.y) + std::abs(o.z) > 1e-9;
  Eigen::Quaterniond q = has_q ? Eigen::Quaterniond(o.w, o.x, o.y, o.z) : Eigen::Quaterniond::Identity();
  if (m.header.frame_id == odom_frame_) {
    // Goal in the odometry frame -> relative to the body (heading frame)
    const auto s = core_.snapshot();
    const double yaw = yaw_of(s.base.orientation);
    const Eigen::Matrix3d r = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    p = r.transpose() * (p - s.base.position);
    q = Eigen::Quaterniond(r.transpose()) * q;
  } else if (!m.header.frame_id.empty() && m.header.frame_id != base_frame_) {
    RCLCPP_WARN(get_logger(), "goal_pose in frame '%s' ignored (use '%s' or '%s')",
      m.header.frame_id.c_str(), base_frame_.c_str(), odom_frame_.c_str());
    return;
  }
  core_.set_pose_goal(p, q);
}

void GaitosNode::on_joy(const sensor_msgs::msg::Joy & m)
{
  RemoteInput r;
  for (std::size_t bit = 0; bit < joy_buttons_.size() && bit < 16; ++bit) {
    const int i = joy_buttons_[bit];
    if (i >= 0 && static_cast<std::size_t>(i) < m.buttons.size() && m.buttons[static_cast<std::size_t>(i)]) {
      r.buttons |= 1u << bit;
    }
  }
  auto axis = [&m](int code) {
      if (code == 0) {
        return 0.0;
      }
      const auto i = static_cast<std::size_t>(std::abs(code) - 1);
      return i < m.axes.size() ? (code > 0 ? 1.0 : -1.0) * m.axes[i] : 0.0;
    };
  // codes are 1-based axis indices with sign (joy: left stick x is axis 0, positive left)
  if (joy_axes_.size() == 4) {
    r.left = Eigen::Vector2d(axis(joy_axes_[0]), axis(joy_axes_[1]));
    r.right = Eigen::Vector2d(axis(joy_axes_[2]), axis(joy_axes_[3]));
  }
  core_.set_remote(r);
}

rclcpp::Time GaitosNode::stamp(const Snapshot & s) const
{
  if (s.simulation && publish_clock_) {
    return rclcpp::Time(static_cast<int64_t>(s.time * 1e9), RCL_ROS_TIME);
  }
  return get_clock()->now();
}

nav_msgs::msg::Odometry GaitosNode::to_msg(const Odometry & o, const rclcpp::Time & t) const
{
  nav_msgs::msg::Odometry m;
  m.header.stamp = t;
  m.header.frame_id = odom_frame_;
  m.child_frame_id = base_frame_;
  m.pose.pose.position.x = o.position.x();
  m.pose.pose.position.y = o.position.y();
  m.pose.pose.position.z = o.position.z();
  m.pose.pose.orientation.w = o.orientation.w();
  m.pose.pose.orientation.x = o.orientation.x();
  m.pose.pose.orientation.y = o.orientation.y();
  m.pose.pose.orientation.z = o.orientation.z();
  m.twist.twist.linear.x = o.linear_velocity.x();
  m.twist.twist.linear.y = o.linear_velocity.y();
  m.twist.twist.linear.z = o.linear_velocity.z();
  m.twist.twist.angular.x = o.angular_velocity.x();
  m.twist.twist.angular.y = o.angular_velocity.y();
  m.twist.twist.angular.z = o.angular_velocity.z();
  return m;
}

void GaitosNode::publish()
{
  if (!status_pub_ || !status_pub_->is_activated()) {
    return;
  }
  const auto s = core_.snapshot();
  if (s.cycle == 0) {
    return;
  }
  const auto t = stamp(s);
  if (clock_pub_) {
    rosgraph_msgs::msg::Clock c;
    c.clock = t;
    clock_pub_->publish(c);
  }
  if (s.mode != last_mode_) {
    std_msgs::msg::String m;
    m.data = s.mode;
    mode_pub_->publish(m);
    last_mode_ = s.mode;
  }
  gaitos_msgs::msg::Status st;
  st.header.stamp = t;
  st.mode = s.mode;
  st.controller = s.controller;
  st.estimate_valid = s.base.valid;
  st.hardware_ok = s.hardware_ok;
  st.dry_run = s.dry_run;
  st.last_safety_event = s.last_safety_event;
  for (std::size_t l = 0; l < core_.model()->leg_count(); ++l) {
    st.legs.push_back(core_.model()->leg(l).name);
    st.contacts.push_back(l < s.contacts.size() && s.contacts.contact[l]);
    st.foot_forces.push_back(l < s.contacts.size() ? s.contacts.force[l] : 0.0);
  }
  status_pub_->publish(st);

  if (publish_joints_) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = t;
    for (std::size_t i = 0; i < core_.model()->joint_count(); ++i) {
      js.name.push_back(core_.model()->joint(i).name);
    }
    js.position = s.joints.position;
    js.velocity = s.joints.velocity;
    js.effort = s.joints.effort;
    joints_pub_->publish(js);
  }

  if (s.base.valid) {
    if (publish_odometry_) {
      odom_pub_->publish(to_msg(s.base.to_odometry(), t));
    }
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = t;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = s.base.position.x();
      tf.transform.translation.y = s.base.position.y();
      tf.transform.translation.z = s.base.position.z();
      tf.transform.rotation.w = s.base.orientation.w();
      tf.transform.rotation.x = s.base.orientation.x();
      tf.transform.rotation.y = s.base.orientation.y();
      tf.transform.rotation.z = s.base.orientation.z();
      tf_->sendTransform(tf);
    }
  }
  const auto gt = s.external_odometry.find("ground_truth");
  if (gt != s.external_odometry.end()) {
    truth_pub_->publish(to_msg(gt->second, t));
  }
}

GaitosNode::CallbackReturn GaitosNode::on_activate(const rclcpp_lifecycle::State & state)
{
  if (!core_.start()) {
    RCLCPP_ERROR(get_logger(), "The hardware failed to start");
    return CallbackReturn::FAILURE;
  }
  LifecycleNode::on_activate(state);
  if (!external_stepping_) {
    running_ = true;
    loop_ = std::thread([this]() {
        try {
          core_.run(running_);
        } catch (const std::exception & e) {
          RCLCPP_FATAL(get_logger(), "Control loop: %s", e.what());
        }
      });
  }
  for (auto & e : extensions_) {
    e->activate();
  }
  RCLCPP_INFO(get_logger(), "Active at %.0f Hz%s", core_.frequency(),
    external_stepping_ ? " (stepped by the host)" : "");
  return CallbackReturn::SUCCESS;
}

GaitosNode::CallbackReturn GaitosNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  for (auto & e : extensions_) {
    e->deactivate();
  }
  running_ = false;
  if (loop_.joinable()) {
    loop_.join();
  }
  core_.stop();
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

GaitosNode::CallbackReturn GaitosNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  extensions_.clear();
  publish_timer_.reset();
  twist_sub_.reset();
  targets_sub_.reset();
  goal_sub_.reset();
  joy_sub_.reset();
  odom_subs_.clear();
  set_mode_srv_.reset();
  mode_srvs_.clear();
  tf_.reset();
  return CallbackReturn::SUCCESS;
}

GaitosNode::CallbackReturn GaitosNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (loop_.joinable()) {
    loop_.join();
  }
  core_.stop();
  return CallbackReturn::SUCCESS;
}

}  // namespace gaitos
