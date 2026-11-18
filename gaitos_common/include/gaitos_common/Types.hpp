//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_COMMON__TYPES_HPP_
#define GAITOS_COMMON__TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gaitos
{

/**
 * @brief Conventions used everywhere in gaitos:
 *  - world frame: z up (ENU-like), gravity (0, 0, -g)
 *  - body frame: x forward, y left, z up (FLU), origin at the base link of the URDF
 *  - joint vectors are indexed with the joint index of the RobotModel
 *  - foot vectors are indexed with the leg index of the RobotModel
 *  - times in seconds, in the clock of the hardware (simulated time in simulation)
 */

/**
 * @struct JointState
 * @brief Measured state of the actuated joints.
 */
struct JointState
{
  std::vector<double> position;   ///< [rad] or [m]
  std::vector<double> velocity;   ///< [rad/s] or [m/s]
  std::vector<double> effort;     ///< [Nm] or [N]

  void resize(std::size_t n);
  std::size_t size() const {return position.size();}
};

/**
 * @struct JointCommand
 * @brief Command of the actuated joints, in the format of the Unitree motors:
 *
 *   tau = kp * (position - q) + kd * (velocity - dq) + effort
 *
 * A joint with kp = kd = 0 is commanded in pure torque; with kp = kd = effort = 0 it is
 * passive (zero torque).
 */
struct JointCommand
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;     ///< Feedforward torque [Nm]
  std::vector<double> kp;         ///< Stiffness [Nm/rad]
  std::vector<double> kd;         ///< Damping [Nm s/rad]

  void resize(std::size_t n);
  std::size_t size() const {return position.size();}

  /**
   * @brief Every joint passive (all zero)
   */
  void set_zero();

  /**
   * @brief Torque that this command produces on a joint at (q, dq)
   */
  double torque(std::size_t i, double q, double dq) const
  {
    return kp[i] * (position[i] - q) + kd[i] * (velocity[i] - dq) + effort[i];
  }
};

/**
 * @struct ImuData
 * @brief Inertial measurement of the base.
 */
struct ImuData
{
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};   ///< Base in world
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};        ///< Body frame [rad/s]
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};     ///< Specific force, body [m/s^2]
  bool has_orientation{true};   ///< false if the device only gives gyro and accelerometer
  double stamp{0.0};
};

/**
 * @struct ContactData
 * @brief Contact of each foot.
 */
struct ContactData
{
  std::vector<double> force;        ///< Normal force (raw sensor or estimated) [N, or sensor units]
  std::vector<bool> contact;        ///< Foot on the ground
  std::vector<double> probability;  ///< Contact probability [0, 1]

  void resize(std::size_t n);
  std::size_t size() const {return contact.size();}
  std::size_t count() const;
};

/**
 * @struct Odometry
 * @brief Odometry of the base, own or from an external source (motion capture, visual or
 * lidar odometry, simulator ground truth...). Pose in the frame of the source,
 * twist in the body frame.
 */
struct Odometry
{
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};    ///< Body frame [m/s]
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};   ///< Body frame [rad/s]
  /// Covariances (diagonal). Negative or zero values: use the noise configured for the source
  Eigen::Vector3d position_variance{Eigen::Vector3d::Constant(-1.0)};
  Eigen::Vector3d velocity_variance{Eigen::Vector3d::Constant(-1.0)};
  double yaw_variance{-1.0};
  bool has_position{true};
  bool has_orientation{true};
  bool has_linear_velocity{true};
  bool has_angular_velocity{true};
};

/**
 * @struct BaseState
 * @brief Estimated state of the floating base.
 */
struct BaseState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};               ///< World [m]
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};  ///< Base in world
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};        ///< World [m/s]
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};       ///< Body [rad/s]
  double stamp{0.0};
  bool valid{false};

  /**
   * @brief Linear velocity in the body frame
   */
  Eigen::Vector3d linear_velocity_body() const
  {
    return orientation.conjugate() * linear_velocity;
  }

  Odometry to_odometry() const;
};

/**
 * @struct BodyTwist
 * @brief Velocity command relative to the body (x forward, y left, yaw rate).
 */
struct BodyTwist
{
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};    ///< [m/s], body frame
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};   ///< [rad/s], body frame
  double stamp{-1.0};   ///< Reception time; negative: never received
};

/**
 * @struct BodyPoseGoal
 * @brief Pose goal relative to the body frame at the moment it is received.
 *
 * Its meaning depends on the operation mode: in a standing mode it is the posture of the
 * body over the feet (height, roll, pitch, yaw); while walking it is a target to walk to.
 */
struct BodyPoseGoal
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double stamp{-1.0};   ///< Reception time; negative: never received
};

/**
 * @struct ControlReference
 * @brief What the active operation mode asks the active controller.
 *
 * Each controller uses the fields it understands: the joint controller tracks `joints`,
 * the stance controller the body pose, the gait controller the body twist and body pose.
 */
struct ControlReference
{
  JointCommand joints;                                              ///< Joint level targets
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};         ///< Body frame [m/s]
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};        ///< Body frame [rad/s]
  Eigen::Vector3d body_offset{Eigen::Vector3d::Zero()};             ///< Over the nominal stance [m]
  Eigen::Vector3d body_rpy{Eigen::Vector3d::Zero()};                ///< Over the nominal stance [rad]
  std::string gait;                                                 ///< Gait name, empty: default
};

/**
 * @struct RemoteInput
 * @brief Hand held remote (gamepad) read by the hardware, e.g. the Unitree wireless
 * controller. Sticks in [-1, 1], buttons as a bit mask (see RemoteButton).
 */
struct RemoteInput
{
  bool valid{false};
  double stamp{-1.0};
  Eigen::Vector2d left{Eigen::Vector2d::Zero()};    ///< (x right, y forward)
  Eigen::Vector2d right{Eigen::Vector2d::Zero()};   ///< (x right, y forward)
  uint32_t buttons{0};
};

/**
 * @brief Button bits of RemoteInput (layout of the Unitree wireless controller)
 */
namespace RemoteButton
{
constexpr uint32_t R1 = 1u << 0;
constexpr uint32_t L1 = 1u << 1;
constexpr uint32_t START = 1u << 2;
constexpr uint32_t SELECT = 1u << 3;
constexpr uint32_t R2 = 1u << 4;
constexpr uint32_t L2 = 1u << 5;
constexpr uint32_t F1 = 1u << 6;
constexpr uint32_t F2 = 1u << 7;
constexpr uint32_t A = 1u << 8;
constexpr uint32_t B = 1u << 9;
constexpr uint32_t X = 1u << 10;
constexpr uint32_t Y = 1u << 11;
constexpr uint32_t UP = 1u << 12;
constexpr uint32_t RIGHT = 1u << 13;
constexpr uint32_t DOWN = 1u << 14;
constexpr uint32_t LEFT = 1u << 15;
}  // namespace RemoteButton

/**
 * @brief Parse a button combination such as "L2+B" into a mask
 * @return 0 if a name is unknown
 */
uint32_t parse_buttons(const std::string & combo);

/**
 * @struct CommandResult
 * @brief Answer to a request (mode change...).
 */
struct CommandResult
{
  bool success{true};
  std::string message;

  static CommandResult ok(const std::string & msg = "") {return {true, msg};}
  static CommandResult fail(const std::string & msg) {return {false, msg};}
};

}  // namespace gaitos

#endif  // GAITOS_COMMON__TYPES_HPP_
