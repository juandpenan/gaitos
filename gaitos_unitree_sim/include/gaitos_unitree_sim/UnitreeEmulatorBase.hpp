//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_UNITREE_SIM__UNITREEEMULATORBASE_HPP_
#define GAITOS_UNITREE_SIM__UNITREEEMULATORBASE_HPP_

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "gaitos_common/Math.hpp"
#include "gaitos_system/NodeExtension.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace gaitos_unitree_sim
{

/// Api ids every Unitree RPC service answers (unitree_sdk2 internal_api.hpp)
namespace api
{
constexpr int64_t kApiVersion = 1;
constexpr int64_t kNoop = 2;
}  // namespace api

/// Status codes of the replies (unitree_api ResponseStatus)
namespace code
{
constexpr int32_t kOk = 0;
constexpr int32_t kNotImplemented = 3203;
constexpr int32_t kBadParameter = 3204;
constexpr int32_t kRejected = 3205;
}  // namespace code

/// Reply to one RPC request
struct Reply
{
  int32_t code{code::kOk};
  std::string data;
};

using RpcHandler = std::function<Reply(int64_t api_id, const std::string & parameter)>;

/// Unitree IMU layout (unitree_go and unitree_hg IMUState): quaternion (w, x, y, z), gyroscope
/// and specific force in the body frame, roll pitch yaw
template<typename Imu>
void fill_imu(const gaitos::ImuData & imu, const gaitos::BaseState & base, Imu & out)
{
  const Eigen::Quaterniond q = imu.has_orientation ? imu.orientation : base.orientation;
  out.quaternion = {static_cast<float>(q.w()), static_cast<float>(q.x()),
    static_cast<float>(q.y()), static_cast<float>(q.z())};
  for (int i = 0; i < 3; ++i) {
    out.gyroscope[static_cast<std::size_t>(i)] = static_cast<float>(imu.angular_velocity(i));
    out.accelerometer[static_cast<std::size_t>(i)] = static_cast<float>(imu.linear_acceleration(i));
  }
  const Eigen::Vector3d rpy = gaitos::quaternion_to_rpy(q);
  out.rpy = {static_cast<float>(rpy.x()), static_cast<float>(rpy.y()), static_cast<float>(rpy.z())};
  out.temperature = 35;
}

/**
 * @class UnitreeEmulatorBase
 * @brief Common part of the emulated DDS interfaces of the Unitree robots (gaitos as the
 * vendor controller, in simulation): the RPC services (`<api_prefix>/<service>/request` ->
 * `/response`, unitree_api, replies matched by identity.id, the internal ids 1 (api version)
 * and 2 (no-op) answered for every service), velocity commands kept until a deadline, the
 * motor slots of the joints and the state timers.
 *
 * Common parameters: api_prefix ("/api"), motor_index (joint -> motor slot; default: the
 * order of the model joints), walking_modes, max_velocity_duration.
 */
class UnitreeEmulatorBase : public gaitos::NodeExtension
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode & node, gaitos::Gaitos & core,
    const gaitos::Params & params) final;
  void deactivate() override;

protected:
  /// Publishers, services and timers of the robot
  virtual void configure_robot(const gaitos::Params & params) = 0;

  /// RPC service `name`; `version` answers api id 1
  void add_service(const std::string & name, const std::string & version, RpcHandler handler);
  void add_timer(double rate, std::function<void()> fn);

  /// Motor slot of each model joint (-1: none), from `motor_index` or the model order
  std::vector<int> motor_slots(const gaitos::Params & params, std::size_t slots) const;

  /// Request a gaitos mode and wait (1 s) for the control loop; ok if already in it
  Reply request_mode(const std::string & mode);
  bool walking() const;   ///< the current mode accepts velocity commands

  /// Body twist (vx, vy, wz) kept fresh until `duration` (clamped to max_velocity_duration)
  void keep_velocity(const std::array<double, 3> & v, double duration);
  void stop_velocity();

  rclcpp_lifecycle::LifecycleNode & node() {return *node_;}
  const rclcpp_lifecycle::LifecycleNode & node() const {return *node_;}
  gaitos::Gaitos & core() {return *core_;}
  const gaitos::Gaitos & core() const {return *core_;}
  rclcpp::QoS state_qos() const {return rclcpp::SensorDataQoS();}

  std::string api_prefix_{"/api"};
  std::vector<std::string> walking_modes_;

private:
  void update_velocity();

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  gaitos::Gaitos * core_{nullptr};
  double max_duration_{1.0};

  std::mutex velocity_mutex_;
  bool velocity_active_{false};
  std::array<double, 3> velocity_{0.0, 0.0, 0.0};
  rclcpp::Time velocity_deadline_;

  rclcpp::CallbackGroup::SharedPtr rpc_group_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> rpc_subs_;
  std::vector<rclcpp::PublisherBase::SharedPtr> rpc_pubs_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

}  // namespace gaitos_unitree_sim

#endif  // GAITOS_UNITREE_SIM__UNITREEEMULATORBASE_HPP_
