//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

// Gazebo (gz-sim) system plugin that hosts the gaitos ROS 2 node inside the simulator, in
// lockstep: Gazebo calls the cycle (step) and the motors are simulated as PD at every
// physics step. All the ROS interfaces are those of GaitosNode (namespace = robot name).
//
//   <plugin filename="gaitos_gz_system" name="gaitos::gazebo::GaitosGzSystem">
//     <ros>
//       <namespace>go2</namespace>
//       <parameters>/path/params.yaml</parameters>   <!-- one or more, in order -->
//     </ros>
//   </plugin>

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <algorithm>
#include <chrono>
#include <thread>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gaitos_common/Logger.hpp"
#include "gaitos_gazebo/GazeboHardware.hpp"
#include "gaitos_system/GaitosNode.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gaitos
{
namespace gazebo
{

using gz::sim::Entity;
using gz::sim::EntityComponentManager;
namespace components = gz::sim::components;

class GaitosGzSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  ~GaitosGzSystem() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_.joinable()) {
      spin_.join();
    }
    if (node_ && node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      node_->deactivate();
    }
  }

  void Configure(
    const Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      logger_.error() << "GaitosGzSystem must be attached to a model";
      return;
    }
    std::vector<std::string> args{"--ros-args", "-p", "use_sim_time:=true"};
    auto element = sdf->Clone();
    std::string ns = model_.Name(ecm);
    if (element->HasElement("ros")) {
      auto ros = element->GetElement("ros");
      if (ros->HasElement("namespace")) {
        ns = ros->Get<std::string>("namespace");
      }
      for (auto e = ros->FindElement("parameters"); e; e = e->GetNextElement("parameters")) {
        args.push_back("--params-file");
        args.push_back(e->Get<std::string>());
      }
    }
    args.push_back("-r");
    args.push_back("__ns:=/" + ns);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
    node_ = std::make_shared<GaitosNode>(rclcpp::NodeOptions().arguments(args));
    hardware_ = std::make_shared<gaitos_gazebo::GazeboHardware>();
    node_->set_hardware(hardware_);
    node_->set_external_stepping(true);
    if (node_->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      logger_.error() << "The gaitos node failed to configure";
      return;
    }
    try {
      const Params hw = Params(node_->get_node_parameters_interface()).sub("hardware");
      period_ = 1.0 / node_->core().frequency();
      ground_truth_ = hw.get<bool>("ground_truth", true);
      contact_fallback_ = hw.get<double>("contact_force_fallback", 30.0);
      configure_entities(hw, ecm);
    } catch (const std::exception & e) {
      logger_.error() << "Configuration error: " << e.what();
      return;
    }
    if (node_->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      logger_.error() << "The gaitos node failed to activate";
      return;
    }
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    executor_->add_node(node_->get_node_base_interface());
    spin_ = std::thread([this]() {executor_->spin();});
    ready_ = true;
    logger_.info() << "gaitos node /" << ns << " running in Gazebo at " << node_->core().frequency() << " Hz";
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, EntityComponentManager & ecm) override
  {
    if (!ready_ || info.paused) {
      return;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    read_sensors(t, ecm);
    if (last_step_ < 0.0 || t < last_step_ || t - last_step_ >= period_ - 1e-9) {
      node_->step();
      last_step_ = t;
    }
    apply_command(ecm);
  }

private:
  void configure_entities(const Params & hw, EntityComponentManager & ecm)
  {
    const auto model = node_->core().model();
    joints_.assign(model->joint_count(), gz::sim::kNullEntity);
    std::string missing;
    for (std::size_t i = 0; i < model->joint_count(); ++i) {
      const Entity j = model_.JointByName(ecm, model->joint(i).name);
      if (j == gz::sim::kNullEntity) {
        missing += model->joint(i).name + " ";
        continue;
      }
      joints_[i] = j;
      if (!ecm.Component<components::JointPosition>(j)) {
        ecm.CreateComponent(j, components::JointPosition());
      }
      if (!ecm.Component<components::JointVelocity>(j)) {
        ecm.CreateComponent(j, components::JointVelocity());
      }
    }
    if (!missing.empty()) {
      logger_.warn() << "Joints not in the Gazebo model (they follow the command): " << missing;
    }
    // Mimic joints of the URDF (not in the model joint vector): they follow their master
    mimic_.clear();
    std::string mimics;
    for (const auto & [name, tj] : model->tree().joints()) {
      if (tj.mimic.empty()) {
        continue;
      }
      const auto master = model->tree().joints().find(tj.mimic);
      const Entity j = model_.JointByName(ecm, name);
      if (j == gz::sim::kNullEntity || master == model->tree().joints().end() ||
        master->second.index < 0)
      {
        continue;
      }
      if (!ecm.Component<components::JointPosition>(j)) {
        ecm.CreateComponent(j, components::JointPosition());
      }
      if (!ecm.Component<components::JointVelocity>(j)) {
        ecm.CreateComponent(j, components::JointVelocity());
      }
      mimic_.push_back({j, static_cast<std::size_t>(master->second.index), tj.mimic_multiplier,
          tj.mimic_offset});
      mimics += name + " ";
    }
    if (!mimics.empty()) {
      logger_.info() << "Mimic joints (follow their master): " << mimics;
    }
    // Initial pose of the joints (SRDF)
    const auto pose = hw.get<std::string>("initial_pose", "");
    if (!pose.empty()) {
      const auto q = model->pose(pose);
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (joints_[i] != gz::sim::kNullEntity) {
          ecm.SetComponentData<components::JointPositionReset>(joints_[i], std::vector<double>{q[i]});
        }
      }
      for (const auto & m : mimic_) {
        ecm.SetComponentData<components::JointPositionReset>(m.joint,
          std::vector<double>{m.multiplier * q[m.master] + m.offset});
      }
      hardware_->input().joints.position = q;
    }
    const auto base = hw.get<std::string>("base_link", model->base_link());
    base_ = gz::sim::Link(model_.LinkByName(ecm, base));
    if (!base_.Valid(ecm)) {
      base_ = gz::sim::Link(model_.CanonicalLink(ecm));
      logger_.warn() << "Base link " << base << " not found, using the canonical link";
    }
    base_.EnableVelocityChecks(ecm, true);
    base_.EnableAccelerationChecks(ecm, true);

    // Foot collisions: those of the configured links (default: the foot link), or those
    // whose name contains the link name (links lumped by the URDF conversion)
    collision_leg_.clear();
    const Params feet = hw.sub("feet");
    std::vector<Entity> all_collisions;
    for (const Entity link : model_.Links(ecm)) {
      for (const Entity c : ecm.ChildrenByComponents(link, components::Collision())) {
        all_collisions.push_back(c);
      }
    }
    for (std::size_t l = 0; l < model->leg_count(); ++l) {
      const auto & leg = model->leg(l);
      auto links = feet.get<std::vector<std::string>>(leg.name + ".links", {leg.foot});
      std::size_t found = 0;
      for (const auto & link_name : links) {
        for (const Entity c : all_collisions) {
          const auto * n = ecm.Component<components::Name>(c);
          const auto * parent = ecm.Component<components::ParentEntity>(c);
          const auto * pname = parent ? ecm.Component<components::Name>(parent->Data()) : nullptr;
          const bool match = (pname && pname->Data() == link_name) ||
            (n && n->Data().find(link_name) != std::string::npos);
          if (match && !collision_leg_.count(c)) {
            collision_leg_[c] = l;
            if (!ecm.Component<components::ContactSensorData>(c)) {
              ecm.CreateComponent(c, components::ContactSensorData());
            }
            ++found;
          }
        }
      }
      if (found == 0) {
        logger_.warn() << "Leg " << leg.name << ": no foot collision found";
      }
    }
  }

  void read_sensors(double t, EntityComponentManager & ecm)
  {
    auto & in = hardware_->input();
    in.time = t;
    const auto & cmd = hardware_->command();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i] == gz::sim::kNullEntity) {
        in.joints.position[i] = cmd.position[i];
        in.joints.velocity[i] = 0.0;
        continue;
      }
      const auto * q = ecm.Component<components::JointPosition>(joints_[i]);
      const auto * dq = ecm.Component<components::JointVelocity>(joints_[i]);
      if (q && !q->Data().empty()) {
        in.joints.position[i] = q->Data()[0];
      }
      if (dq && !dq->Data().empty()) {
        in.joints.velocity[i] = dq->Data()[0];
      }
      in.joints.effort[i] = effort_.size() > i ? effort_[i] : 0.0;
    }
    const auto pose = base_.WorldPose(ecm);
    if (pose) {
      const auto & r = pose->Rot();
      const Eigen::Quaterniond q(r.W(), r.X(), r.Y(), r.Z());
      in.imu.orientation = q;
      in.imu.has_orientation = true;
      in.imu.stamp = t;
      const auto w = base_.WorldAngularVelocity(ecm);
      const auto v = base_.WorldLinearVelocity(ecm);
      const auto a = base_.WorldLinearAcceleration(ecm);
      const Eigen::Vector3d w_world = w ? Eigen::Vector3d(w->X(), w->Y(), w->Z()) : Eigen::Vector3d::Zero();
      const Eigen::Vector3d v_world = v ? Eigen::Vector3d(v->X(), v->Y(), v->Z()) : Eigen::Vector3d::Zero();
      const Eigen::Vector3d a_world = a ? Eigen::Vector3d(a->X(), a->Y(), a->Z()) : Eigen::Vector3d::Zero();
      in.imu.angular_velocity = q.conjugate() * w_world;
      in.imu.linear_acceleration = q.conjugate() * (a_world + Eigen::Vector3d(0.0, 0.0, 9.81));
      in.has_ground_truth = ground_truth_;
      in.ground_truth.stamp = t;
      in.ground_truth.position = Eigen::Vector3d(pose->Pos().X(), pose->Pos().Y(), pose->Pos().Z());
      in.ground_truth.orientation = q;
      in.ground_truth.linear_velocity = q.conjugate() * v_world;
      in.ground_truth.angular_velocity = in.imu.angular_velocity;
    }
    std::fill(in.foot_force.begin(), in.foot_force.end(), 0.0);
    for (const auto & [collision, leg] : collision_leg_) {
      const auto * data = ecm.Component<components::ContactSensorData>(collision);
      if (!data) {
        continue;
      }
      for (const auto & contact : data->Data().contact()) {
        if (contact.wrench_size() == 0) {
          in.foot_force[leg] += contact_fallback_ * std::max(1, contact.position_size());
          continue;
        }
        for (const auto & w : contact.wrench()) {
          const auto & f1 = w.body_1_wrench().force();
          const auto & f2 = w.body_2_wrench().force();
          in.foot_force[leg] += std::max(std::hypot(f1.x(), f1.y(), f1.z()), std::hypot(f2.x(), f2.y(), f2.z()));
        }
      }
    }
  }

  void apply_command(EntityComponentManager & ecm)
  {
    const auto & cmd = hardware_->command();
    const auto & in = hardware_->input();
    const auto model = node_->core().model();
    effort_.resize(joints_.size(), 0.0);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i] == gz::sim::kNullEntity) {
        continue;
      }
      double tau = cmd.torque(i, in.joints.position[i], in.joints.velocity[i]);
      const double limit = model->joint(i).effort;
      if (limit > 0.0) {
        tau = std::clamp(tau, -limit, limit);
      }
      effort_[i] = tau;
      set_force(ecm, joints_[i], tau);
    }
    for (const auto & m : mimic_) {
      const auto i = m.master;
      if (i >= cmd.position.size() || i >= cmd.kp.size()) {
        continue;
      }
      const auto * q = ecm.Component<components::JointPosition>(m.joint);
      const auto * dq = ecm.Component<components::JointVelocity>(m.joint);
      if (!q || q->Data().empty() || !dq || dq->Data().empty()) {
        continue;
      }
      const double target = m.multiplier * cmd.position[i] + m.offset;
      const double target_v = i < cmd.velocity.size() ? m.multiplier * cmd.velocity[i] : 0.0;
      double tau = cmd.kp[i] * (target - q->Data()[0]) +
        (i < cmd.kd.size() ? cmd.kd[i] : 0.0) * (target_v - dq->Data()[0]);
      const double limit = model->joint(i).effort;
      if (limit > 0.0) {
        tau = std::clamp(tau, -limit, limit);
      }
      set_force(ecm, m.joint, tau);
    }
  }

  static void set_force(EntityComponentManager & ecm, Entity joint, double tau)
  {
    auto * f = ecm.Component<components::JointForceCmd>(joint);
    if (f) {
      f->Data() = {tau};
    } else {
      ecm.CreateComponent(joint, components::JointForceCmd({tau}));
    }
  }

  std::shared_ptr<GaitosNode> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_;
  std::shared_ptr<gaitos_gazebo::GazeboHardware> hardware_;
  gz::sim::Model model_;
  gz::sim::Link base_;
  std::vector<Entity> joints_;
  struct Mimic
  {
    Entity joint;
    std::size_t master;
    double multiplier;
    double offset;
  };
  std::vector<Mimic> mimic_;
  std::vector<double> effort_;
  std::map<Entity, std::size_t> collision_leg_;
  Logger logger_{"gaitos.gazebo"};
  double period_{0.002};
  double last_step_{-1.0};
  double contact_fallback_{30.0};
  bool ground_truth_{true};
  bool ready_{false};
};

}  // namespace gazebo
}  // namespace gaitos

GZ_ADD_PLUGIN(
  gaitos::gazebo::GaitosGzSystem, gz::sim::System,
  gaitos::gazebo::GaitosGzSystem::ISystemConfigure,
  gaitos::gazebo::GaitosGzSystem::ISystemPreUpdate)
