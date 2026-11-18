//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_unitree/UnitreeHardware.hpp"

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <chrono>
#include <cstring>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace gaitos_unitree
{

using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelSubscriber;

namespace
{
constexpr const char * kLowCmd = "rt/lowcmd";
constexpr const char * kLowState = "rt/lowstate";
constexpr const char * kSportState = "rt/sportmodestate";
constexpr float kPosStop = 2.146e9f;
constexpr float kVelStop = 16000.0f;

/// CRC of the low level commands (unitree_sdk2 examples)
uint32_t crc32_core(const uint32_t * ptr, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  const uint32_t poly = 0x04c11db7;
  for (uint32_t i = 0; i < len; ++i) {
    uint32_t xbit = 1u << 31;
    const uint32_t data = ptr[i];
    for (int bits = 0; bits < 32; ++bits) {
      if (crc & 0x80000000) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
      if (data & xbit) {
        crc ^= poly;
      }
      xbit >>= 1;
    }
  }
  return crc;
}

double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

template<typename Imu>
void copy_imu(const Imu & src, gaitos::ImuData & imu, double stamp)
{
  const auto & q = src.quaternion();   // w, x, y, z
  imu.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
  imu.angular_velocity = Eigen::Vector3d(src.gyroscope()[0], src.gyroscope()[1], src.gyroscope()[2]);
  imu.linear_acceleration =
    Eigen::Vector3d(src.accelerometer()[0], src.accelerometer()[1], src.accelerometer()[2]);
  imu.has_orientation = true;
  imu.stamp = stamp;
}
}  // namespace

// ------------------------------------------------------------------------------------------

bool UnitreeHardwareBase::on_initialize()
{
  interface_ = params().get<std::string>("network_interface", interface_);
  domain_id_ = params().get<int>("domain_id", domain_id_);
  release_sport_mode_ = params().get<bool>("release_sport_mode", release_sport_mode_);
  state_timeout_ = params().get<double>("state_timeout", state_timeout_);
  foot_force_scale_ = params().get<double>("foot_force_scale", foot_force_scale_);
  sport_odometry_ = params().get<bool>("sport_odometry", sport_odometry_);
  emergency_kd_ = params().get<double>("emergency_kd", emergency_kd_);
  const auto map = params().get_map("motor_index");
  motor_index_.resize(model().joint_count());
  for (std::size_t i = 0; i < model().joint_count(); ++i) {
    const auto it = map.find(model().joint(i).name);
    motor_index_[i] = it != map.end() ? static_cast<int>(it->second) : static_cast<int>(i);
  }
  return true;
}

void UnitreeHardwareBase::init_channels()
{
  static std::once_flag once;
  std::call_once(once, [this]() {
      unitree::robot::ChannelFactory::Instance()->Init(domain_id_, interface_);
      logger().info() << "DDS on " << interface_ << ", domain " << domain_id_;
    });
}

void UnitreeHardwareBase::release_motion_service()
{
  if (!release_sport_mode_) {
    return;
  }
  unitree::robot::b2::MotionSwitcherClient msc;
  msc.SetTimeout(5.0f);
  msc.Init();
  std::string form, name;
  for (int attempt = 0; attempt < 10; ++attempt) {
    if (msc.CheckMode(form, name) != 0) {
      logger().warn() << "Motion switcher not available (simulator?)";
      return;
    }
    if (name.empty()) {
      logger().info() << "Built-in motion service released";
      return;
    }
    logger().info() << "Releasing built-in motion service '" << name << "'";
    msc.ReleaseMode();
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  logger().error() << "Could not release the built-in motion service";
}

gaitos::RemoteInput UnitreeHardwareBase::parse_remote(const uint8_t * data, double stamp)
{
  // unitree_sdk2 example gamepad.hpp: head[2], keys (uint16), lx, rx, ry, L2, ly (float)
  gaitos::RemoteInput r;
  uint16_t keys = 0;
  float lx = 0.0f, rx = 0.0f, ry = 0.0f, ly = 0.0f;
  std::memcpy(&keys, data + 2, sizeof(keys));
  std::memcpy(&lx, data + 4, sizeof(float));
  std::memcpy(&rx, data + 8, sizeof(float));
  std::memcpy(&ry, data + 12, sizeof(float));
  std::memcpy(&ly, data + 20, sizeof(float));
  r.valid = true;
  r.stamp = stamp;
  r.buttons = keys;
  r.left = Eigen::Vector2d(lx, ly);
  r.right = Eigen::Vector2d(rx, ry);
  return r;
}

bool UnitreeHardwareBase::state_fresh() const
{
  return last_state_ > 0.0 && steady_now() - last_state_ < state_timeout_;
}

// ------------------------------------------------------------------------------------------
// unitree_go (Go2)

struct UnitreeGoHardware::Impl
{
  unitree_go::msg::dds_::LowState_ state;
  unitree_go::msg::dds_::LowCmd_ cmd;
  unitree_go::msg::dds_::SportModeState_ sport;
  uint64_t sport_seq{0};
  std::shared_ptr<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>> pub;
  std::shared_ptr<ChannelSubscriber<unitree_go::msg::dds_::LowState_>> sub;
  std::shared_ptr<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sport_sub;
};

UnitreeGoHardware::UnitreeGoHardware()
: impl_(std::make_unique<Impl>()) {}
UnitreeGoHardware::~UnitreeGoHardware() = default;

bool UnitreeGoHardware::start()
{
  init_channels();
  release_motion_service();
  auto & c = impl_->cmd;
  c.head()[0] = 0xFE;
  c.head()[1] = 0xEF;
  c.level_flag() = 0xFF;
  c.gpio() = 0;
  for (auto & m : c.motor_cmd()) {
    m.mode() = 0x01;
    m.q() = kPosStop;
    m.dq() = kVelStop;
    m.kp() = 0.0f;
    m.kd() = 0.0f;
    m.tau() = 0.0f;
  }
  impl_->pub = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(kLowCmd);
  impl_->pub->InitChannel();
  impl_->sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(kLowState);
  impl_->sub->InitChannel([this](const void * msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      impl_->state = *static_cast<const unitree_go::msg::dds_::LowState_ *>(msg);
      last_state_ = steady_now();
    }, 1);
  if (sport_odometry_) {
    impl_->sport_sub =
      std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(kSportState);
    impl_->sport_sub->InitChannel([this](const void * msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        impl_->sport = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);
        ++impl_->sport_seq;
      }, 1);
  }
  // Wait for the first state
  for (int i = 0; i < 200 && !state_fresh(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!state_fresh()) {
    logger().error() << "No " << kLowState << " received on " << interface_ << " (domain " << domain_id_ << ")";
    return false;
  }
  return true;
}

void UnitreeGoHardware::stop()
{
  if (impl_->pub && write_enabled()) {
    // Damped joints before leaving
    for (std::size_t i = 0; i < motor_index_.size(); ++i) {
      auto & m = impl_->cmd.motor_cmd()[static_cast<std::size_t>(motor_index_[i])];
      m.q() = kPosStop;
      m.dq() = 0.0f;          // velocity target 0: damping (kVelStop would disable it)
      m.kp() = 0.0f;
      m.kd() = static_cast<float>(emergency_kd_);
      m.tau() = 0.0f;
    }
    impl_->cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&impl_->cmd),
        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    impl_->pub->Write(impl_->cmd);
  }
  impl_->sub.reset();
  impl_->sport_sub.reset();
  impl_->pub.reset();
}

void UnitreeGoHardware::emergency_stop()
{
  if (!impl_->pub || !write_enabled()) {
    return;
  }
  unitree_go::msg::dds_::LowCmd_ cmd;
  cmd.head()[0] = 0xFE;
  cmd.head()[1] = 0xEF;
  cmd.level_flag() = 0xFF;
  for (auto & m : cmd.motor_cmd()) {
    m.mode() = 0x01;
    m.q() = kPosStop;
    m.dq() = 0.0f;
    m.kp() = 0.0f;
    m.kd() = static_cast<float>(emergency_kd_);
    m.tau() = 0.0f;
  }
  cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&cmd), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  impl_->pub->Write(cmd);
}

bool UnitreeGoHardware::read(gaitos::RobotState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & s = impl_->state;
  const double t = now();
  for (std::size_t i = 0; i < motor_index_.size(); ++i) {
    const auto & m = s.motor_state()[static_cast<std::size_t>(motor_index_[i])];
    state.joints.position[i] = m.q();
    state.joints.velocity[i] = m.dq();
    state.joints.effort[i] = m.tau_est();
  }
  copy_imu(s.imu_state(), state.imu_raw, t);
  state.remote = parse_remote(s.wireless_remote().data(), t);
  state.foot_force_raw.resize(model().leg_count());
  for (std::size_t l = 0; l < model().leg_count() && l < 4; ++l) {
    state.foot_force_raw[l] = foot_force_scale_ * s.foot_force()[l];
  }
  if (sport_odometry_ && impl_->sport_seq > 0) {
    auto & ext = state.external_odometry["unitree_sport"];
    if (ext.seq != impl_->sport_seq) {
      const auto & sp = impl_->sport;
      ext.odometry.stamp = t;
      ext.odometry.position = Eigen::Vector3d(sp.position()[0], sp.position()[1], sp.position()[2]);
      const auto & q = sp.imu_state().quaternion();
      ext.odometry.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
      ext.odometry.linear_velocity = ext.odometry.orientation.conjugate() *
        Eigen::Vector3d(sp.velocity()[0], sp.velocity()[1], sp.velocity()[2]);
      ext.odometry.has_angular_velocity = false;
      ext.reception = t;
      ext.seq = impl_->sport_seq;
    }
  }
  return state_fresh();
}

bool UnitreeGoHardware::write(const gaitos::JointCommand & command)
{
  if (!write_enabled()) {
    return true;
  }
  auto & c = impl_->cmd;
  for (std::size_t i = 0; i < motor_index_.size(); ++i) {
    auto & m = c.motor_cmd()[static_cast<std::size_t>(motor_index_[i])];
    m.mode() = 0x01;
    m.q() = static_cast<float>(command.position[i]);
    m.dq() = static_cast<float>(command.velocity[i]);
    m.kp() = static_cast<float>(command.kp[i]);
    m.kd() = static_cast<float>(command.kd[i]);
    m.tau() = static_cast<float>(command.effort[i]);
  }
  c.crc() = crc32_core(reinterpret_cast<uint32_t *>(&c), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  return impl_->pub && impl_->pub->Write(c);
}

// ------------------------------------------------------------------------------------------
// unitree_hg (R1, G1, H1-2)

struct UnitreeHgHardware::Impl
{
  unitree_hg::msg::dds_::LowState_ state;
  unitree_hg::msg::dds_::LowCmd_ cmd;
  std::shared_ptr<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> pub;
  std::shared_ptr<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> sub;
};

UnitreeHgHardware::UnitreeHgHardware()
: impl_(std::make_unique<Impl>()) {}
UnitreeHgHardware::~UnitreeHgHardware() = default;

bool UnitreeHgHardware::start()
{
  init_channels();
  release_motion_service();
  impl_->pub = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(kLowCmd);
  impl_->pub->InitChannel();
  impl_->sub = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(kLowState);
  impl_->sub->InitChannel([this](const void * msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      impl_->state = *static_cast<const unitree_hg::msg::dds_::LowState_ *>(msg);
      last_state_ = steady_now();
    }, 1);
  for (int i = 0; i < 200 && !state_fresh(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!state_fresh()) {
    logger().error() << "No " << kLowState << " received on " << interface_ << " (domain " << domain_id_ << ")";
    return false;
  }
  for (auto & m : impl_->cmd.motor_cmd()) {
    m.mode() = 0;
    m.kp() = 0.0f;
    m.kd() = 0.0f;
  }
  return true;
}

void UnitreeHgHardware::stop()
{
  if (impl_->pub && write_enabled()) {
    for (std::size_t i = 0; i < motor_index_.size(); ++i) {
      auto & m = impl_->cmd.motor_cmd()[static_cast<std::size_t>(motor_index_[i])];
      m.kp() = 0.0f;
      m.kd() = 3.0f;
      m.tau() = 0.0f;
      m.dq() = 0.0f;
    }
    impl_->cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&impl_->cmd),
        (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
    impl_->pub->Write(impl_->cmd);
  }
  impl_->sub.reset();
  impl_->pub.reset();
}

void UnitreeHgHardware::emergency_stop()
{
  if (!impl_->pub || !write_enabled()) {
    return;
  }
  unitree_hg::msg::dds_::LowCmd_ cmd;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd.mode_machine() = impl_->state.mode_machine();
  }
  cmd.mode_pr() = 0;
  for (std::size_t i = 0; i < motor_index_.size(); ++i) {
    auto & m = cmd.motor_cmd()[static_cast<std::size_t>(motor_index_[i])];
    m.mode() = 1;
    m.kp() = 0.0f;
    m.kd() = static_cast<float>(emergency_kd_);
    m.tau() = 0.0f;
    m.dq() = 0.0f;
  }
  cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&cmd), (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
  impl_->pub->Write(cmd);
}

bool UnitreeHgHardware::read(gaitos::RobotState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & s = impl_->state;
  for (std::size_t i = 0; i < motor_index_.size(); ++i) {
    const auto & m = s.motor_state()[static_cast<std::size_t>(motor_index_[i])];
    state.joints.position[i] = m.q();
    state.joints.velocity[i] = m.dq();
    state.joints.effort[i] = m.tau_est();
  }
  copy_imu(s.imu_state(), state.imu_raw, now());
  state.remote = parse_remote(s.wireless_remote().data(), now());
  state.foot_force_raw.clear();   // no foot sensors: TorqueContact
  impl_->cmd.mode_machine() = s.mode_machine();
  return state_fresh();
}

bool UnitreeHgHardware::write(const gaitos::JointCommand & command)
{
  if (!write_enabled()) {
    return true;
  }
  auto & c = impl_->cmd;
  c.mode_pr() = 0;   // ankles in pitch / roll (series) mode
  for (std::size_t i = 0; i < motor_index_.size(); ++i) {
    auto & m = c.motor_cmd()[static_cast<std::size_t>(motor_index_[i])];
    m.mode() = 1;
    m.q() = static_cast<float>(command.position[i]);
    m.dq() = static_cast<float>(command.velocity[i]);
    m.kp() = static_cast<float>(command.kp[i]);
    m.kd() = static_cast<float>(command.kd[i]);
    m.tau() = static_cast<float>(command.effort[i]);
  }
  c.crc() = crc32_core(reinterpret_cast<uint32_t *>(&c), (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
  return impl_->pub && impl_->pub->Write(c);
}

}  // namespace gaitos_unitree

PLUGINLIB_EXPORT_CLASS(gaitos_unitree::UnitreeGoHardware, gaitos::HardwareInterface)
PLUGINLIB_EXPORT_CLASS(gaitos_unitree::UnitreeHgHardware, gaitos::HardwareInterface)
