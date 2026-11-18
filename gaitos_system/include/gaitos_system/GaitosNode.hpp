//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_SYSTEM__GAITOSNODE_HPP_
#define GAITOS_SYSTEM__GAITOSNODE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "gaitos_msgs/msg/status.hpp"
#include "gaitos_msgs/srv/set_mode.hpp"
#include "gaitos_system/Gaitos.hpp"
#include "gaitos_system/NodeExtension.hpp"
#include "pluginlib/class_loader.hpp"

namespace gaitos
{

/**
 * @class GaitosNode
 * @brief ROS 2 lifecycle node of gaitos. The node namespace is the robot name.
 *
 * Lifecycle: configure loads the model (robot_description, robot_description_semantic) and
 * every plugin (pluginlib) from the parameters; activate starts the hardware and the
 * real-time loop thread (or waits for the host to call step(), e.g. the Gazebo plugin);
 * deactivate leaves the robot in the safe mode.
 *
 * Interfaces (relative to the namespace):
 *  - subscriptions: cmd_vel (geometry_msgs/Twist, body frame), goal_pose
 *    (geometry_msgs/PoseStamped, relative to the body if frame_id is the base frame or empty,
 *    in the odometry frame if it is odom_frame), joint_targets (sensor_msgs/JointState:
 *    positions, and max speeds in `velocity` (optional), of joints moved over the controller,
 *    see Gaitos::set_joint_targets), external odometries (nav_msgs/Odometry,
 *    `external_odometry.sources` / `.topics`), joy (sensor_msgs/Joy, `teleop.joy.*`)
 *  - services: set_mode (gaitos_msgs/SetMode), mode/<name> (std_srvs/Trigger) per mode
 *  - publishers: mode (std_msgs/String, transient local), status (gaitos_msgs/Status),
 *    odometry (nav_msgs/Odometry), joint_states (sensor_msgs/JointState), ground_truth
 *    (nav_msgs/Odometry, simulators), tf odom -> base, /clock (simulators run by the node);
 *    `publish.joint_states`, `publish.odometry` and `publish_tf` turn off the robot state
 *    when another node publishes it (e.g. a vendor stack fed by an extension)
 *  - extensions: NodeExtension plugins with more interfaces (`extensions.<name>.plugin`)
 */
class GaitosNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit GaitosNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GaitosNode() override;

  /// Backend created by the host (call before configure)
  void set_hardware(std::shared_ptr<HardwareInterface> hardware) {hardware_ = std::move(hardware);}
  /// The host runs the cycles with step() (call before activate)
  void set_external_stepping(bool external) {external_stepping_ = external;}

  /// One cycle (external stepping)
  bool step() {return core_.step();}

  Gaitos & core() {return core_;}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void create_interfaces();
  bool load_extensions();
  void publish();
  CommandResult request(const std::string & mode);
  void on_goal(const geometry_msgs::msg::PoseStamped & msg);
  void on_joy(const sensor_msgs::msg::Joy & msg);
  rclcpp::Time stamp(const Snapshot & s) const;
  nav_msgs::msg::Odometry to_msg(const Odometry & o, const rclcpp::Time & t) const;

  Gaitos core_;
  std::shared_ptr<HardwareInterface> hardware_;
  bool external_stepping_{false};
  std::atomic<bool> running_{false};
  std::thread loop_;

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  bool publish_joints_{true};
  bool publish_odometry_{true};
  bool publish_clock_{false};
  std::string last_mode_;

  // Joy mapping
  std::vector<int> joy_buttons_;
  std::vector<int> joy_axes_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr targets_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Service<gaitos_msgs::srv::SetMode>::SharedPtr set_mode_srv_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> mode_srvs_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<gaitos_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Extensions: the loader outlives the instances it creates
  std::unique_ptr<pluginlib::ClassLoader<NodeExtension>> extension_loader_;
  std::vector<std::shared_ptr<NodeExtension>> extensions_;
};

}  // namespace gaitos

#endif  // GAITOS_SYSTEM__GAITOSNODE_HPP_
