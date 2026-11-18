//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_model/RobotModel.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>

namespace gaitos
{

std::shared_ptr<RobotModel> RobotModel::from_params(const Params & root)
{
  const Params robot = root.sub("robot");
  auto description = root.get<std::string>("robot_description", "");
  if (description.empty()) {
    description = read_file(robot.require<std::string>("urdf"));
  }
  KinematicTree tree = parse_urdf_string(description);
  SrdfData srdf;
  auto semantic = root.get<std::string>("robot_description_semantic", "");
  if (semantic.empty() && robot.has("srdf")) {
    semantic = read_file(robot.require<std::string>("srdf"));
  }
  if (!semantic.empty()) {
    srdf = parse_srdf_string(semantic, tree);
  }
  const auto name = robot.get<std::string>("name", srdf.robot_name.empty() ? "robot" : srdf.robot_name);
  return create(name, std::move(tree), std::move(srdf), robot);
}

std::shared_ptr<RobotModel> RobotModel::create(
  const std::string & name, KinematicTree tree, SrdfData srdf, const Params & robot)
{
  auto model = std::make_shared<RobotModel>();
  model->name_ = name;
  model->tree_ = std::move(tree);
  model->srdf_ = std::move(srdf);
  model->base_link_ = robot.get<std::string>("base_link", model->tree_.root());
  if (!model->tree_.has_link(model->base_link_)) {
    throw std::runtime_error("Unknown base link " + model->base_link_);
  }

  // Actuated joints and their order
  std::vector<std::string> joint_names = robot.get<std::vector<std::string>>("joints", {});
  if (joint_names.empty()) {
    const auto all = model->srdf_.groups.find("all");
    joint_names = all != model->srdf_.groups.end() ? all->second.joints : model->tree_.movable_joints();
  }
  for (const auto & n : joint_names) {
    auto it = model->tree_.joints().find(n);
    if (it == model->tree_.joints().end() || !it->second.movable()) {
      throw std::runtime_error("Joint " + n + " is not a movable joint of the URDF");
    }
    if (model->joint_map_.count(n)) {
      throw std::runtime_error("Joint " + n + " listed twice");
    }
    it->second.index = static_cast<int>(model->joints_.size());
    model->joint_map_[n] = model->joints_.size();
    model->joints_.push_back({n, it->second.lower, it->second.upper, it->second.effort,
        it->second.velocity});
  }

  // Legs: parallel lists (ROS parameters have no lists of maps)
  const auto groups = robot.get<std::vector<std::string>>("legs.groups", {});
  const auto names = robot.get<std::vector<std::string>>("legs.names", groups);
  const auto orientation = robot.get<std::vector<bool>>("legs.orientation_ik", {});
  if (names.size() != groups.size()) {
    throw std::runtime_error("robot.legs.names and robot.legs.groups must have the same size");
  }
  std::set<std::size_t> leg_joints;
  for (std::size_t i = 0; i < groups.size(); ++i) {
    LegInfo info;
    info.group = groups[i];
    info.name = names[i];
    info.orientation_ik = i < orientation.size() && orientation[i];
    const auto g = model->srdf_.groups.find(info.group);
    if (g == model->srdf_.groups.end() || g->second.chain_tip.empty()) {
      throw std::runtime_error("Leg " + info.name + ": SRDF group " + info.group +
              " must exist and be a chain");
    }
    info.foot = g->second.chain_tip;
    info.chain = model->tree_.chain(model->base_link_, info.foot);
    for (const int idx : info.chain.joint_indices()) {
      leg_joints.insert(static_cast<std::size_t>(idx));
    }
    model->legs_.push_back(std::move(info));
  }
  for (std::size_t i = 0; i < model->joints_.size(); ++i) {
    if (!leg_joints.count(i)) {
      model->upper_body_.push_back(i);
    }
  }
  model->mass_ = robot.get<double>("mass", model->tree_.mass());
  return model;
}

std::optional<std::size_t> RobotModel::joint_index(const std::string & name) const
{
  const auto it = joint_map_.find(name);
  if (it == joint_map_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<std::size_t> RobotModel::indices(const std::vector<std::string> & names) const
{
  std::vector<std::size_t> out;
  for (const auto & n : names) {
    const auto idx = joint_index(n);
    if (!idx) {
      throw std::runtime_error("Joint " + n + " is not an actuated joint of " + name_);
    }
    out.push_back(*idx);
  }
  return out;
}

std::vector<std::size_t> RobotModel::group_indices(const std::string & group) const
{
  if (group == "all") {
    std::vector<std::size_t> out(joints_.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = i;
    }
    return out;
  }
  if (group == "legs") {
    std::vector<std::size_t> out;
    for (const auto & leg : legs_) {
      for (const int idx : leg.chain.joint_indices()) {
        out.push_back(static_cast<std::size_t>(idx));
      }
    }
    return out;
  }
  if (group == "upper_body") {
    return upper_body_;
  }
  const auto it = srdf_.groups.find(group);
  if (it == srdf_.groups.end()) {
    throw std::runtime_error("Unknown joint group " + group);
  }
  return indices(it->second.joints);
}

std::vector<std::string> RobotModel::pose_names() const
{
  std::vector<std::string> out;
  for (const auto & [name, pose] : srdf_.poses) {
    (void)pose;
    out.push_back(name);
  }
  return out;
}

std::vector<double> RobotModel::pose(const std::string & name, const std::vector<double> & base) const
{
  const auto it = srdf_.poses.find(name);
  if (it == srdf_.poses.end()) {
    throw std::runtime_error("Robot " + name_ + " has no pose '" + name + "' in its SRDF");
  }
  std::vector<double> q = base.size() == joints_.size() ? base : std::vector<double>(joints_.size(), 0.0);
  for (const auto & [joint, value] : it->second) {
    const auto idx = joint_index(joint);
    if (idx) {
      q[*idx] = value;
    }
  }
  return q;
}

void RobotModel::clamp_to_limits(std::vector<double> & q) const
{
  for (std::size_t i = 0; i < q.size() && i < joints_.size(); ++i) {
    q[i] = std::clamp(q[i], joints_[i].lower, joints_[i].upper);
  }
}

}  // namespace gaitos
