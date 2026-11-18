//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_model/Kinematics.hpp"

#include <algorithm>
#include <stdexcept>

#include "gaitos_common/Math.hpp"

namespace gaitos
{

Eigen::Isometry3d TreeJoint::transform(double q) const
{
  Eigen::Isometry3d t = origin;
  switch (type) {
    case JointType::REVOLUTE:
    case JointType::CONTINUOUS:
      t.rotate(Eigen::AngleAxisd(q, axis));
      break;
    case JointType::PRISMATIC:
      t.translate(axis * q);
      break;
    case JointType::FIXED:
      break;
  }
  return t;
}

KinematicChain::KinematicChain(std::string base, std::string tip, std::vector<Segment> segments)
: base_(std::move(base)), tip_(std::move(tip)), segments_(std::move(segments))
{
  for (auto & s : segments_) {
    if (s.type != JointType::FIXED) {
      if (s.index < 0) {
        throw std::runtime_error(
                "Chain " + base_ + " -> " + tip_ + " has a movable joint that is not actuated");
      }
      const auto it = std::find(indices_.begin(), indices_.end(), s.index);
      s.column = static_cast<int>(it - indices_.begin());
      if (it == indices_.end()) {
        indices_.push_back(s.index);
      }
    }
  }
}

namespace
{
double value_of(const KinematicChain::Segment & s, const std::vector<double> & q)
{
  return s.index >= 0 ? s.multiplier * q.at(static_cast<std::size_t>(s.index)) + s.offset : 0.0;
}

Eigen::Isometry3d segment_transform(const KinematicChain::Segment & s, double q)
{
  Eigen::Isometry3d t = s.origin;
  if (s.type == JointType::REVOLUTE || s.type == JointType::CONTINUOUS) {
    t.rotate(Eigen::AngleAxisd(q, s.axis));
  } else if (s.type == JointType::PRISMATIC) {
    t.translate(s.axis * q);
  }
  return t;
}
}  // namespace

Eigen::Isometry3d KinematicChain::forward(const std::vector<double> & q) const
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (const auto & s : segments_) {
    t = t * segment_transform(s, value_of(s, q));
  }
  return t;
}

Eigen::MatrixXd KinematicChain::jacobian(const std::vector<double> & q, Eigen::Isometry3d * tip) const
{
  // Joint axes and origins in the base frame, then the tip pose
  struct Frame
  {
    Eigen::Vector3d axis;
    Eigen::Vector3d origin;
    const Segment * segment;
  };
  std::vector<Frame> frames;
  frames.reserve(segments_.size());
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (const auto & s : segments_) {
    const Eigen::Isometry3d joint_frame = t * s.origin;
    if (s.type != JointType::FIXED) {
      frames.push_back({joint_frame.linear() * s.axis, joint_frame.translation(), &s});
    }
    t = t * segment_transform(s, value_of(s, q));
  }
  Eigen::MatrixXd j = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(indices_.size()));
  for (const auto & f : frames) {
    const auto c = static_cast<Eigen::Index>(f.segment->column);
    const double m = f.segment->multiplier;
    if (f.segment->type == JointType::PRISMATIC) {
      j.block<3, 1>(0, c) += m * f.axis;
    } else {
      j.block<3, 1>(0, c) += m * f.axis.cross(t.translation() - f.origin);
      j.block<3, 1>(3, c) += m * f.axis;
    }
  }
  if (tip) {
    *tip = t;
  }
  return j;
}

double KinematicChain::inverse(
  const Eigen::Isometry3d & target, std::vector<double> & q, const IkOptions & options) const
{
  const auto n = static_cast<Eigen::Index>(indices_.size());
  std::vector<std::pair<double, double>> limits(indices_.size(), {-1e9, 1e9});
  for (const auto & s : segments_) {
    if (s.type != JointType::FIXED && s.multiplier == 1.0 && s.offset == 0.0) {
      auto & l = limits[static_cast<std::size_t>(s.column)];
      l.first = std::max(l.first, s.lower);
      l.second = std::min(l.second, s.upper);
    }
  }
  const int rows = options.orientation ? 6 : 3;
  double error = 0.0;
  for (int it = 0; it < options.max_iterations; ++it) {
    Eigen::Isometry3d tip;
    const Eigen::MatrixXd jac = jacobian(q, &tip);
    Eigen::VectorXd e(rows);
    e.head<3>() = target.translation() - tip.translation();
    error = e.head<3>().norm();
    Eigen::MatrixXd j = jac.topRows(rows);
    if (options.orientation) {
      e.tail<3>() = options.orientation_weight * rotation_error(target.linear(), tip.linear());
      j.bottomRows(3) *= options.orientation_weight;
      error = std::max(error, e.tail<3>().norm());
    }
    if (error < options.tolerance) {
      break;
    }
    const Eigen::MatrixXd jjt = j * j.transpose() +
      options.damping * Eigen::MatrixXd::Identity(rows, rows);
    Eigen::VectorXd dq = j.transpose() * jjt.ldlt().solve(e);
    const double max_abs = dq.cwiseAbs().maxCoeff();
    if (max_abs > options.max_step) {
      dq *= options.max_step / max_abs;
    }
    for (Eigen::Index k = 0; k < n; ++k) {
      auto & v = q[static_cast<std::size_t>(indices_[static_cast<std::size_t>(k)])];
      const auto & lim = limits[static_cast<std::size_t>(k)];
      v = std::clamp(v + dq(k), lim.first, lim.second);
    }
  }
  return error;
}

Eigen::Matrix<double, 6, 1> KinematicChain::tip_velocity(
  const std::vector<double> & q, const std::vector<double> & dq) const
{
  const Eigen::MatrixXd j = jacobian(q);
  Eigen::VectorXd v(static_cast<Eigen::Index>(indices_.size()));
  for (std::size_t k = 0; k < indices_.size(); ++k) {
    v(static_cast<Eigen::Index>(k)) = dq.at(static_cast<std::size_t>(indices_[k]));
  }
  return j * v;
}

void KinematicTree::finalize()
{
  std::vector<std::string> roots;
  for (const auto & [name, link] : links_) {
    (void)link;
    roots.push_back(name);
  }
  for (auto & [name, joint] : joints_) {
    auto child = links_.find(joint.child);
    if (child == links_.end() || links_.find(joint.parent) == links_.end()) {
      throw std::runtime_error("Joint " + name + " connects unknown links");
    }
    child->second.parent_joint = name;
    roots.erase(std::remove(roots.begin(), roots.end(), joint.child), roots.end());
  }
  if (roots.size() != 1) {
    throw std::runtime_error("The kinematic tree must have exactly one root, it has " +
            std::to_string(roots.size()));
  }
  root_ = roots.front();
  // Topological order
  order_.clear();
  order_.push_back(root_);
  for (std::size_t i = 0; i < order_.size(); ++i) {
    for (const auto & [name, joint] : joints_) {
      (void)name;
      if (joint.parent == order_[i]) {
        order_.push_back(joint.child);
      }
    }
  }
}

std::vector<std::string> KinematicTree::chain_joints(const std::string & base, const std::string & tip) const
{
  std::vector<std::string> joints;
  std::string link = tip;
  while (link != base) {
    const auto it = links_.find(link);
    if (it == links_.end() || it->second.parent_joint.empty()) {
      throw std::runtime_error("Link " + base + " is not an ancestor of " + tip);
    }
    const auto & joint = joints_.at(it->second.parent_joint);
    joints.push_back(joint.name);
    link = joint.parent;
  }
  std::reverse(joints.begin(), joints.end());
  return joints;
}

KinematicChain KinematicTree::chain(const std::string & base, const std::string & tip) const
{
  std::vector<KinematicChain::Segment> segments;
  for (const auto & name : chain_joints(base, tip)) {
    const auto & j = joints_.at(name);
    KinematicChain::Segment seg{j.origin, j.type, j.axis, j.movable() ? j.index : -1, j.lower,
      j.upper};
    if (j.movable() && j.index < 0 && !j.mimic.empty() && joints_.count(j.mimic)) {
      seg.index = joints_.at(j.mimic).index;
      seg.multiplier = j.mimic_multiplier;
      seg.offset = j.mimic_offset;
    }
    segments.push_back(seg);
  }
  return KinematicChain(base, tip, segments);
}

double KinematicTree::joint_value(const TreeJoint & j, const std::vector<double> & q) const
{
  if (j.index >= 0 && static_cast<std::size_t>(j.index) < q.size()) {
    return q[static_cast<std::size_t>(j.index)];
  }
  if (!j.mimic.empty()) {
    const auto it = joints_.find(j.mimic);
    if (it != joints_.end()) {
      return j.mimic_multiplier * joint_value(it->second, q) + j.mimic_offset;
    }
  }
  return 0.0;
}

std::map<std::string, Eigen::Isometry3d> KinematicTree::link_poses(const std::vector<double> & q) const
{
  std::map<std::string, Eigen::Isometry3d> poses;
  poses[root_] = Eigen::Isometry3d::Identity();
  for (std::size_t i = 1; i < order_.size(); ++i) {
    const auto & link = links_.at(order_[i]);
    const auto & joint = joints_.at(link.parent_joint);
    poses[link.name] = poses.at(joint.parent) * joint.transform(joint_value(joint, q));
  }
  return poses;
}

std::vector<std::string> KinematicTree::movable_joints() const
{
  std::vector<std::string> out;
  for (std::size_t i = 1; i < order_.size(); ++i) {
    const auto & joint = joints_.at(links_.at(order_[i]).parent_joint);
    if (joint.movable() && joint.mimic.empty()) {
      out.push_back(joint.name);
    }
  }
  return out;
}

double KinematicTree::mass() const
{
  double m = 0.0;
  for (const auto & [name, link] : links_) {
    (void)name;
    m += link.mass;
  }
  return m;
}

Eigen::Vector3d KinematicTree::center_of_mass(const std::vector<double> & q) const
{
  const auto poses = link_poses(q);
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double m = 0.0;
  for (const auto & [name, link] : links_) {
    sum += link.mass * (poses.at(name) * link.com);
    m += link.mass;
  }
  return m > 0.0 ? Eigen::Vector3d(sum / m) : Eigen::Vector3d::Zero();
}

RigidBodyInertia KinematicTree::rigid_body(const std::vector<double> & q, const std::string & frame) const
{
  const auto poses = link_poses(q);
  const Eigen::Isometry3d to_frame = frame.empty() ? Eigen::Isometry3d::Identity() : poses.at(frame).inverse();
  RigidBodyInertia rb;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto & [name, link] : links_) {
    sum += link.mass * (to_frame * poses.at(name) * link.com);
    rb.mass += link.mass;
  }
  if (rb.mass <= 0.0) {
    return rb;
  }
  rb.com = sum / rb.mass;
  for (const auto & [name, link] : links_) {
    const Eigen::Isometry3d t = to_frame * poses.at(name);
    const Eigen::Matrix3d r = t.linear();
    const Eigen::Vector3d d = t * link.com - rb.com;
    rb.inertia += r * link.inertia * r.transpose() +
      link.mass * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
  }
  return rb;
}

}  // namespace gaitos
