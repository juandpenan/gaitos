//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_NAOQI_SIM__NAOQIDRIVEREMULATOR_HPP_
#define GAITOS_NAOQI_SIM__NAOQIDRIVEREMULATOR_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gaitos_system/NodeExtension.hpp"
#include "naoqi_bridge_msgs/msg/joint_angles_with_speed.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace gaitos_naoqi_sim
{

/**
 * @class NaoqiDriverEmulator
 * @brief The ROS 2 interface of naoqi_driver (ros-naoqi naoqi_driver2) in front of a simulated
 * gaitos NAO, so that a program written for the real NAO runs unchanged against MuJoCo or
 * Gazebo. gaitos plays the role of NAOqi ALMotion: its posture modes, its stance and its walk.
 *
 * Publishes (as naoqi_driver, names in `topics.*`):
 *  - /joint_states (sensor_msgs/JointState, `rates.joint_states` 50 Hz): every joint of the
 *    body, mimic ones (RHipYawPitch) included; the link TF comes from a robot_state_publisher
 *  - TF odom -> base_link (the gaitos estimate) and base_link -> base_footprint (on the ground
 *    under the torso, yaw only), at the joint state rate
 *  - /odom (nav_msgs/Odometry odom -> base_link, `rates.odom` 15 Hz)
 *  - /imu/torso (sensor_msgs/Imu in base_link, `rates.imu` 10 Hz)
 *
 * Subscribes:
 *  - /cmd_vel (geometry_msgs/Twist): ALMotion.move, the velocity holds until the next message
 *    (zero stops). Only while standing or walking (`walking_modes`; from `stand_modes` the
 *    stance mode `stand_mode` is entered first); otherwise ignored, as on a resting robot.
 *  - /joint_angles (naoqi_bridge_msgs/JointAnglesWithSpeed): ALMotion.setAngles (relative 0)
 *    and changeAngles (relative 1): joint names or one chain (`chains`: Body, Head, LArm,
 *    RArm, LLeg, RLeg), speed as a fraction of the max speed of each joint. Through
 *    Gaitos::set_joint_targets, so the joints allowed by `external_joints` of the robot.
 *  - /move_base_simple/goal (geometry_msgs/PoseStamped in odom or base_footprint):
 *    ALMotion.moveTo, a walk to that pose (the walking mode `walk_mode` with a pose goal).
 *  - /speech (std_msgs/String): logged (no audio in simulation).
 * Not emulated: cameras, sonars, bumpers, touch sensors, audio, info, diagnostics.
 */
class NaoqiDriverEmulator : public gaitos::NodeExtension
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode & node, gaitos::Gaitos & core,
    const gaitos::Params & params) override;
  void deactivate() override;

  // Pieces without ROS, public for the tests
  /// JointAnglesWithSpeed -> joint targets; empty if the names and angles do not match
  std::vector<gaitos::JointTarget> joint_targets(
    const naoqi_bridge_msgs::msg::JointAnglesWithSpeed & msg) const;
  /// ALMotion.move: true if accepted (the robot stands or walks)
  bool move(double vx, double vy, double wz);
  /// ALMotion.moveTo with a pose relative to the body (x, y, yaw): true if accepted
  bool move_to(double x, double y, double yaw);
  sensor_msgs::msg::JointState joint_state(const gaitos::Snapshot & s) const;

private:
  void on_goal(const geometry_msgs::msg::PoseStamped & msg);
  void publish_fast();
  void publish_odom();
  void publish_imu();
  void keep_velocity();
  bool in(const std::vector<std::string> & modes, const std::string & mode) const;
  bool ensure_mode(const std::string & mode);
  rclcpp::Time stamp() const {return node_->now();}

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  gaitos::Gaitos * core_{nullptr};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string footprint_frame_{"base_footprint"};
  std::vector<std::string> walking_modes_;
  std::vector<std::string> stand_modes_;
  std::string stand_mode_;
  std::string walk_mode_;
  std::map<std::string, std::vector<std::string>> chains_;   ///< NAOqi chain -> joints
  std::vector<std::string> sole_links_;
  double default_speed_{0.1};

  // Mimic joints published in joint_states: name, master index, multiplier, offset
  struct Mimic
  {
    std::string name;
    std::size_t master;
    double multiplier;
    double offset;
  };
  std::vector<Mimic> mimics_;

  std::mutex velocity_mutex_;
  bool moving_{false};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>::SharedPtr angles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr speech_sub_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

}  // namespace gaitos_naoqi_sim

#endif  // GAITOS_NAOQI_SIM__NAOQIDRIVEREMULATOR_HPP_
