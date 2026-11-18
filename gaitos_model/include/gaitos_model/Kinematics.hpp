//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_MODEL__KINEMATICS_HPP_
#define GAITOS_MODEL__KINEMATICS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <map>
#include <string>
#include <vector>

namespace gaitos
{

/**
 * @enum JointType
 * @brief Joint types of the kinematic tree (floating and planar joints are not part of it:
 * the floating base is the root of the tree).
 */
enum class JointType {FIXED, REVOLUTE, CONTINUOUS, PRISMATIC};

/**
 * @struct TreeJoint
 * @brief Joint of the kinematic tree, as in the URDF.
 */
struct TreeJoint
{
  std::string name;
  JointType type{JointType::FIXED};
  std::string parent;
  std::string child;
  Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};   ///< Child frame in parent at q = 0
  Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};
  double lower{-1e9};
  double upper{1e9};
  double effort{0.0};
  double velocity{0.0};
  std::string mimic;            ///< Joint this one mimics (not actuated)
  double mimic_multiplier{1.0};
  double mimic_offset{0.0};
  int index{-1};                ///< Index in the model joint vectors, -1 if not actuated

  bool movable() const {return type != JointType::FIXED;}

  /**
   * @brief Transform of the child frame in the parent frame at position q
   */
  Eigen::Isometry3d transform(double q) const;
};

/**
 * @struct TreeLink
 * @brief Link of the kinematic tree, with its inertia (if any).
 */
struct TreeLink
{
  std::string name;
  std::string parent_joint;     ///< Empty for the root
  double mass{0.0};
  Eigen::Vector3d com{Eigen::Vector3d::Zero()};   ///< In the link frame
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
};

/**
 * @class KinematicChain
 * @brief Serial chain of joints from a base link to a tip link (a leg, an arm).
 *
 * Joint values are read from (and written to) vectors indexed with the model joint
 * indices, so the same vector holds all the joints of the robot.
 */
class KinematicChain
{
public:
  struct Segment
  {
    Eigen::Isometry3d origin;
    JointType type;
    Eigen::Vector3d axis;
    int index;          ///< Model index (of the master joint for mimic joints), -1 if fixed
    double lower;
    double upper;
    double multiplier{1.0};   ///< Mimic joints: q = multiplier * q_master + offset
    double offset{0.0};
    int column{-1};           ///< Column in the jacobian (set by the chain)
  };

  KinematicChain() = default;
  KinematicChain(std::string base, std::string tip, std::vector<Segment> segments);

  const std::string & base() const {return base_;}
  const std::string & tip() const {return tip_;}

  /**
   * @brief Model indices of the movable joints of the chain, from base to tip
   */
  const std::vector<int> & joint_indices() const {return indices_;}
  std::size_t dof() const {return indices_.size();}

  /**
   * @brief Pose of the tip in the base frame
   */
  Eigen::Isometry3d forward(const std::vector<double> & q) const;

  /**
   * @brief Geometric jacobian of the tip, in the base frame (rows: linear, angular;
   * columns: joints of the chain in order)
   * @param[out] tip pose of the tip (optional)
   */
  Eigen::MatrixXd jacobian(const std::vector<double> & q, Eigen::Isometry3d * tip = nullptr) const;

  /**
   * @struct IkOptions
   * @brief Damped least squares inverse kinematics settings
   */
  struct IkOptions
  {
    bool orientation{false};          ///< Also match the orientation of the target
    double orientation_weight{0.3};   ///< [m/rad] weight of the orientation error
    int max_iterations{30};
    double tolerance{1e-5};           ///< [m]
    double damping{1e-3};
    double max_step{0.2};             ///< Max joint change per iteration [rad]
  };

  /**
   * @brief Inverse kinematics from the current values of q (warm start), respecting the
   * joint limits
   * @param target desired pose of the tip in the base frame (only the position unless
   * options.orientation)
   * @param[in,out] q model joint vector; only the joints of the chain are changed
   * @return final position error [m]
   */
  double inverse(const Eigen::Isometry3d & target, std::vector<double> & q, const IkOptions & options) const;

  /**
   * @brief Velocity of the tip (linear, angular) in the base frame, given the joint velocities
   */
  Eigen::Matrix<double, 6, 1> tip_velocity(
    const std::vector<double> & q, const std::vector<double> & dq) const;

private:
  std::string base_;
  std::string tip_;
  std::vector<Segment> segments_;
  std::vector<int> indices_;
};

/**
 * @struct RigidBodyInertia
 * @brief The whole robot seen as one rigid body (single rigid body models: MPC, LIPM).
 */
struct RigidBodyInertia
{
  double mass{0.0};
  Eigen::Vector3d com{Eigen::Vector3d::Zero()};       ///< In the reference frame
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};   ///< About the CoM, reference frame axes
};

/**
 * @class KinematicTree
 * @brief Links and joints of a URDF.
 */
class KinematicTree
{
public:
  void add_link(const TreeLink & link) {links_[link.name] = link;}
  void add_joint(const TreeJoint & joint) {joints_[joint.name] = joint;}

  /**
   * @brief Find the root and order the links; call after adding everything
   * @throws std::runtime_error if the tree is not connected or has several roots
   */
  void finalize();

  const std::string & root() const {return root_;}
  const std::map<std::string, TreeLink> & links() const {return links_;}
  const std::map<std::string, TreeJoint> & joints() const {return joints_;}
  std::map<std::string, TreeJoint> & joints() {return joints_;}
  bool has_link(const std::string & name) const {return links_.count(name) > 0;}

  /**
   * @brief Chain between two links (base must be an ancestor of tip)
   * @throws std::runtime_error if there is no such chain
   */
  KinematicChain chain(const std::string & base, const std::string & tip) const;

  /**
   * @brief Movable joints between two links, in order
   */
  std::vector<std::string> chain_joints(const std::string & base, const std::string & tip) const;

  /**
   * @brief Pose of every link in the root frame
   * @param q model joint vector (mimic joints are computed from their master)
   */
  std::map<std::string, Eigen::Isometry3d> link_poses(const std::vector<double> & q) const;

  /**
   * @brief Movable joints that do not mimic another one, in topological order
   */
  std::vector<std::string> movable_joints() const;

  /**
   * @brief Total mass [kg]
   */
  double mass() const;

  /**
   * @brief Center of mass in the root frame
   */
  Eigen::Vector3d center_of_mass(const std::vector<double> & q) const;

  /**
   * @brief Composite rigid body of the whole robot at configuration q
   * @param frame link whose frame expresses the result (empty: root)
   */
  RigidBodyInertia rigid_body(const std::vector<double> & q, const std::string & frame = "") const;

private:
  double joint_value(const TreeJoint & j, const std::vector<double> & q) const;

  std::map<std::string, TreeLink> links_;
  std::map<std::string, TreeJoint> joints_;
  std::string root_;
  std::vector<std::string> order_;   ///< Links in topological order (root first)
};

}  // namespace gaitos

#endif  // GAITOS_MODEL__KINEMATICS_HPP_
