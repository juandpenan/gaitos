//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_system/Gaitos.hpp"

#include <algorithm>
#include <chrono>
#include <set>
#include <thread>

#include "gaitos_core/Gains.hpp"
#include "gaitos_system/RealTime.hpp"

namespace gaitos
{

Gaitos::Gaitos() = default;

Gaitos::~Gaitos()
{
  if (started_) {
    stop();
  }
}

void Gaitos::configure(const Params & config, std::shared_ptr<HardwareInterface> hardware)
{
  config_ = config;

  // Robot (robot_description, robot_description_semantic, robot.*)
  model_ = RobotModel::from_params(config);
  state_ = RobotState();
  state_.name = config.get<std::string>("name", model_->name());
  state_.resize(model_->joint_count(), model_->leg_count());
  logger_.info() << "Robot " << model_->name() << ": " << model_->joint_count() << " joints, " <<
    model_->leg_count() << " legs, " << model_->mass() << " kg, poses: " <<
    [this]() {
      std::string s;
      for (const auto & p : model_->pose_names()) {
        s += p + " ";
      }
      return s;
    }();

  frequency_ = config.get<double>("frequency", frequency_);
  max_hardware_failures_ = config.get<int>("max_hardware_failures", max_hardware_failures_);
  real_time_factor_ = config.get<double>("real_time_factor", real_time_factor_);

  // Hardware
  const Params hw = config.sub("hardware");
  if (hardware) {
    hardware_ = std::move(hardware);
    if (!hardware_->initialize("hardware", hw, model_)) {
      throw std::runtime_error("The hardware failed to initialize");
    }
  } else {
    hardware_ = hardware_loader_.create("hardware", hw, model_);
  }
  state_.simulation = hardware_->is_simulation();

  // Actuators: every joint must be handled by one of them
  actuators_.clear();
  const Params act = config.sub("actuators");
  std::vector<int> owner(model_->joint_count(), -1);
  auto add_actuator = [&](const std::string & name, const Params & p) {
      auto a = actuator_loader_.create(name, p, model_, "gaitos_actuators/PdPassthrough");
      a->set_joints(read_joint_selection(p, "joints", *model_));
      for (const auto j : a->joints()) {
        if (owner[j] >= 0) {
          throw std::runtime_error("Joint " + model_->joint(j).name + " is in two actuators");
        }
        owner[j] = static_cast<int>(actuators_.size());
      }
      actuators_.push_back(a);
    };
  if (act.valid()) {
    for (const auto & name : act.keys()) {
      add_actuator(name, act.sub(name));
    }
  } else {
    add_actuator("actuators", config.sub("actuators.default"));
  }
  for (std::size_t j = 0; j < owner.size(); ++j) {
    if (owner[j] < 0) {
      throw std::runtime_error("Joint " + model_->joint(j).name + " has no actuator");
    }
  }

  // Sensors and estimator (defaults if not configured)
  imu_ = imu_loader_.create("imu", config.sub("imu"), model_, "gaitos_sensors/ImuPassthrough");
  contacts_ = contact_loader_.create(
    "contacts", config.sub("contacts"), model_, "gaitos_sensors/ForceThresholdContact");
  estimator_ = estimator_loader_.create(
    "estimator", config.sub("estimator"), model_, "gaitos_estimators/KalmanEstimator");

  // Controllers
  controllers_.clear();
  const Params ctrl = config.sub("controllers");
  for (const auto & name : ctrl.keys()) {
    controllers_[name] = controller_loader_.create(name, ctrl.sub(name), model_);
  }
  if (!controllers_.count("joint")) {
    controllers_["joint"] = controller_loader_.create(
      "joint", ctrl.sub("joint"), model_, "gaitos_controllers/JointPdController");
  }

  // Modes
  modes_.configure(config, model_);

  // Safety
  const Params safety = config.sub("safety");
  dry_run_ = config.get<bool>("dry_run", false);
  safety_.configure(safety.sub("limits"), model_);
  teleop_.configure(config.sub("teleop"));
  gain_ramp_time_ = safety.get<double>("gain_ramp_time", 0.0);
  ramp_from_ = safety.get<std::vector<std::string>>("ramp_from", {modes_.safe_mode(), "zero_torque"});
  watchdog_timeout_ = safety.get<double>("watchdog_timeout", 0.0);

  // Joints that applications may move over the controller (external targets)
  const Params ext = config.sub("external_joints");
  external_group_.assign(model_->joint_count(), false);
  for (const auto & group : ext.get<std::vector<std::string>>("groups", {})) {
    for (const auto i : model_->group_indices(group)) {
      external_group_[i] = true;
    }
  }
  external_any_modes_ = ext.get<std::vector<std::string>>("any_joint_modes", {});
  external_off_modes_ = ext.get<std::vector<std::string>>("off_modes",
      {modes_.safe_mode(), "zero_torque"});
  external_.assign(model_->joint_count(), ExternalJoint());
  external_mode_.clear();
  hardware_->set_write_enabled(!dry_run_);
  if (dry_run_) {
    logger_.warn() << "DRY RUN: the hardware is read but no command is sent";
  }
  for (const auto & c : modes_.controllers()) {
    if (!controllers_.count(c)) {
      throw std::runtime_error("A mode uses controller '" + c + "', which is not configured");
    }
  }
  logger_.info() << "Configured: " << actuators_.size() << " actuator group(s), " <<
    controllers_.size() << " controller(s), " << modes_.modes().size() << " modes";
}

bool Gaitos::start()
{
  if (!hardware_) {
    return false;
  }
  if (!hardware_->start()) {
    logger_.error() << "The hardware failed to start";
    return false;
  }
  started_ = true;
  first_cycle_ = true;
  return true;
}

void Gaitos::stop()
{
  if (!started_) {
    return;
  }
  modes_.force_safe(state_, "stopping");
  // A few cycles in the safe mode so the command reaches the robot
  if (!hardware_->paces_loop()) {
    for (int i = 0; i < 3; ++i) {
      step();
    }
  }
  hardware_->stop();
  started_ = false;
}

ControllerBase * Gaitos::controller(const std::string & name) const
{
  const auto it = controllers_.find(name);
  return it == controllers_.end() ? nullptr : it->second.get();
}

std::future<CommandResult> Gaitos::request_mode(const std::string & mode)
{
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  mode_requests_.emplace_back(mode, std::promise<CommandResult>());
  return mode_requests_.back().second.get_future();
}

void Gaitos::set_twist(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular)
{
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  twist_.linear = linear;
  twist_.angular = angular;
  has_twist_ = true;
}

void Gaitos::set_pose_goal(const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  pose_goal_.position = position;
  pose_goal_.orientation = orientation.normalized();
  has_pose_goal_ = true;
}

void Gaitos::push_odometry(const std::string & source, const Odometry & odometry)
{
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  odometry_inbox_[source] = odometry;
}

void Gaitos::set_remote(const RemoteInput & remote)
{
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  remote_inbox_ = remote;
  remote_inbox_.valid = true;
  has_remote_ = true;
}

std::size_t Gaitos::set_joint_targets(const std::vector<JointTarget> & targets)
{
  std::size_t known = 0;
  for (const auto & t : targets) {
    known += model_ && model_->joint_index(t.name) ? 1 : 0;
  }
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  external_inbox_.insert(external_inbox_.end(), targets.begin(), targets.end());
  return known;
}

bool Gaitos::external_joint_allowed(std::size_t joint, const std::string & mode) const
{
  if (joint >= external_group_.size() ||
    std::find(external_off_modes_.begin(), external_off_modes_.end(), mode) != external_off_modes_.end())
  {
    return false;
  }
  return external_group_[joint] ||
         std::find(external_any_modes_.begin(), external_any_modes_.end(), mode) != external_any_modes_.end();
}

void Gaitos::apply_external_joints()
{
  std::vector<JointTarget> inbox;
  {
    std::lock_guard<std::mutex> lock(inbox_mutex_);
    inbox.swap(external_inbox_);
  }
  if (external_.empty()) {
    return;
  }
  const std::string & mode = state_.mode;
  if (mode != external_mode_) {   // a new mode (posture, gait...) takes the joints back
    for (auto & e : external_) {
      e.active = false;
    }
    external_mode_ = mode;
  }
  for (const auto & t : inbox) {
    const auto idx = model_->joint_index(t.name);
    if (!idx || !external_joint_allowed(*idx, mode)) {
      continue;
    }
    const auto i = *idx;
    auto & e = external_[i];
    if (!e.active) {   // start from what the controller commands now
      e.setpoint = state_.command.position[i];
      e.target = e.setpoint;
    }
    const auto & info = model_->joint(i);
    e.target = std::clamp((t.relative ? e.target : 0.0) + t.position, info.lower, info.upper);
    const double vmax = info.velocity > 0.0 ? info.velocity : 1.0;
    e.speed = t.speed > 0.0 ? std::min(t.speed, vmax) : vmax;
    e.active = true;
  }
  for (std::size_t i = 0; i < external_.size(); ++i) {
    auto & e = external_[i];
    if (!e.active) {
      continue;
    }
    if (!external_joint_allowed(i, mode)) {
      e.active = false;
      continue;
    }
    const double step = e.speed * state_.dt;
    e.setpoint += std::clamp(e.target - e.setpoint, -step, step);
    state_.command.position[i] = e.setpoint;
    state_.command.velocity[i] = 0.0;
    state_.command.effort[i] = 0.0;
  }
}

void Gaitos::set_mode_callback(ModeManager::ChangeCallback cb)
{
  modes_.set_change_callback(std::move(cb));
}

void Gaitos::apply_commands()
{
  std::lock_guard<std::mutex> lock(inbox_mutex_);
  if (has_twist_) {
    state_.twist_command = twist_;
    state_.twist_command.stamp = state_.time;
    has_twist_ = false;
  }
  if (has_pose_goal_) {
    state_.pose_goal = pose_goal_;
    state_.pose_goal.stamp = state_.time;
    ++state_.pose_goal_seq;
    has_pose_goal_ = false;
  }
  for (auto & [source, odom] : odometry_inbox_) {
    auto & ext = state_.external_odometry[source];
    ext.odometry = odom;
    ext.reception = state_.time;
    ++ext.seq;
  }
  odometry_inbox_.clear();
  if (has_remote_) {
    remote_ = remote_inbox_;
    remote_.stamp = state_.time;
    has_remote_ = false;
  }
}

bool Gaitos::step()
{
  const double now = hardware_->now();
  state_.dt = first_cycle_ ? 1.0 / frequency_ : std::max(0.0, now - last_time_);
  first_cycle_ = false;
  last_time_ = now;
  state_.time = now;
  ++state_.cycle;

  apply_commands();
  const bool hw_ok = hardware_->read(state_);
  hardware_failures_ = hw_ok ? 0 : hardware_failures_ + 1;
  if (hardware_failures_ > max_hardware_failures_) {
    modes_.force_safe(state_, "hardware read failed");
  }
  // Hand held remote: an external one (joy) while it is fresh, else the one of the hardware
  if (remote_.valid && now - remote_.stamp < 0.5) {
    state_.remote = remote_;
  }
  // Emergency stop first
  const auto remote = teleop_.update(state_.remote, now);
  if (remote.estop && modes_.current() != modes_.safe_mode()) {
    last_safety_event_ = "emergency stop (remote)";
    modes_.force_safe(state_, last_safety_event_);
  }
  if (remote.twist) {
    state_.twist_command.linear = remote.linear;
    state_.twist_command.angular = remote.angular;
    state_.twist_command.stamp = now;
  }
  // Joint limits (outside the safe mode)
  if (modes_.current() != modes_.safe_mode() && hw_ok) {
    const auto violation = safety_.check(state_.joints);
    if (!violation.empty()) {
      last_safety_event_ = "joint limits: " + violation;
      logger_.error() << last_safety_event_;
      modes_.force_safe(state_, last_safety_event_);
    }
  } else {
    safety_.reset();
  }
  if (!imu_->update(state_.imu_raw, state_.dt, state_.imu)) {
    state_.imu.has_orientation = false;
  }
  contacts_->update(state_, state_.contacts);
  BaseState estimate = state_.base;
  if (estimator_->update(state_, estimate) || !estimate.valid) {
    state_.base = estimate;
  }

  // Mode requests after the estimate, so that can_enter sees the current state
  std::vector<std::pair<std::string, std::promise<CommandResult>>> requests;
  {
    std::lock_guard<std::mutex> lock(inbox_mutex_);
    requests.swap(mode_requests_);
  }
  if (!remote.estop && !remote.mode.empty()) {
    const auto r = modes_.request(remote.mode, state_);
    if (!r.success) {
      logger_.warn() << "Remote mode request rejected: " << r.message;
    }
  }
  for (auto & [mode, promise] : requests) {
    const auto r = modes_.request(mode, state_);
    if (!r.success) {
      logger_.warn() << "Mode request rejected: " << r.message;
    }
    promise.set_value(r);
  }

  const std::string & controller_name = modes_.update(state_, state_.reference);
  state_.mode = modes_.current();
  auto & controller = controllers_.at(controller_name);
  if (controller_name != state_.controller) {
    controller->activate(state_);
    state_.controller = controller_name;
  }
  if (!controller->update(state_, state_.reference, state_.dt, state_.command)) {
    logger_.error() << "Controller " << controller_name << " failed";
    modes_.force_safe(state_, "controller failed");
    state_.command.set_zero();
    state_.command.position = state_.joints.position;
    std::fill(state_.command.kd.begin(), state_.command.kd.end(), 1.0);
  }

  apply_external_joints();

  // Gain ramp when the robot leaves a passive mode: stiffness and feedforward from 0
  if (state_.mode != previous_mode_) {
    const bool from_passive = std::find(ramp_from_.begin(), ramp_from_.end(), previous_mode_) !=
      ramp_from_.end();
    ramp_start_ = (from_passive && gain_ramp_time_ > 0.0) ? now : -1.0;
    previous_mode_ = state_.mode;
  }
  if (ramp_start_ >= 0.0) {
    const double scale = std::clamp((now - ramp_start_) / gain_ramp_time_, 0.0, 1.0);
    for (std::size_t i = 0; i < state_.command.size(); ++i) {
      state_.command.kp[i] *= scale;
      state_.command.effort[i] *= scale;
    }
    if (scale >= 1.0) {
      ramp_start_ = -1.0;
    }
  }

  state_.motor_command = state_.command;
  for (auto & a : actuators_) {
    a->update(state_.joints, state_.command, state_.motor_command);
  }
  const bool write_ok = dry_run_ ? true : hardware_->write(state_.motor_command);
  last_cycle_wall_ = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.time = state_.time;
    snapshot_.cycle = state_.cycle;
    snapshot_.mode = state_.mode;
    snapshot_.controller = state_.controller;
    snapshot_.base = state_.base;
    snapshot_.imu = state_.imu;
    snapshot_.contacts = state_.contacts;
    snapshot_.joints = state_.joints;
    snapshot_.command = state_.motor_command;
    for (const auto & [name, ext] : state_.external_odometry) {
      snapshot_.external_odometry[name] = ext.odometry;
    }
    snapshot_.hardware_ok = hw_ok && write_ok;
    snapshot_.dry_run = dry_run_;
    snapshot_.simulation = state_.simulation;
    snapshot_.last_safety_event = last_safety_event_;
  }
  return hw_ok && write_ok;
}

Snapshot Gaitos::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

void Gaitos::run(const std::atomic<bool> & running)
{
  const Params rt = config_.sub("real_time");
  if (rt.get<bool>("enabled", false)) {
    const bool required = rt.get<bool>("required", false);
    auto check = [&](const std::string & err) {
        if (err.empty()) {
          return;
        }
        if (required) {
          throw std::runtime_error("[real_time] " + err);
        }
        logger_.warn() << "[real_time] " << err << ", running without it";
      };
    if (rt.get<bool>("lock_memory", true)) {
      check(lock_memory());
    }
    const int cpu = rt.get<int>("cpu", -1);
    if (cpu >= 0) {
      check(set_cpu_affinity(cpu));
    }
    check(set_real_time_priority(rt.get<int>("priority", 80)));
  }

  // Watchdog: if no cycle completes in watchdog_timeout, stop the robot from another thread
  std::atomic<bool> watching{watchdog_timeout_ > 0.0 && !dry_run_};
  last_cycle_wall_ = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  std::thread watchdog;
  if (watching) {
    watchdog = std::thread([this, &watching, &running]() {
        bool fired = false;
        while (watching && running) {
          std::this_thread::sleep_for(std::chrono::duration<double>(watchdog_timeout_ / 4.0));
          const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
          const bool stalled = now - last_cycle_wall_ > watchdog_timeout_;
          if (stalled && !fired) {
            logger_.error() << "Watchdog: no cycle for " << now - last_cycle_wall_ <<
              " s, emergency stop of the hardware";
            hardware_->emergency_stop();
            fired = true;
          } else if (!stalled) {
            fired = false;
          }
        }
      });
  }
  struct StopWatchdog
  {
    std::atomic<bool> & flag;
    std::thread & thread;
    ~StopWatchdog()
    {
      flag = false;
      if (thread.joinable()) {
        thread.join();
      }
    }
  } stop_watchdog{watching, watchdog};

  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clock::duration>(
    std::chrono::duration<double>(1.0 / frequency_));
  auto next = clock::now();
  const auto wall_start = clock::now();
  const double sim_start = hardware_->now();
  auto last_report = clock::now();
  while (running) {
    step();
    if (hardware_->paces_loop()) {
      // Lockstep simulation: keep simulated time at real_time_factor x wall time (0: max speed)
      if (real_time_factor_ > 0.0) {
        const double sim_elapsed = hardware_->now() - sim_start;
        const auto target = wall_start + std::chrono::duration_cast<clock::duration>(
          std::chrono::duration<double>(sim_elapsed / real_time_factor_));
        std::this_thread::sleep_until(target);
      }
      continue;
    }
    next += period;
    const auto now = clock::now();
    if (now > next) {
      ++overruns_;
      next = now;
    } else {
      std::this_thread::sleep_until(next);
    }
    if (now - last_report > std::chrono::seconds(10) && overruns_ > 0) {
      logger_.warn() << overruns_ << " cycles over the period in the last 10 s";
      overruns_ = 0;
      last_report = now;
    }
  }
}

}  // namespace gaitos
