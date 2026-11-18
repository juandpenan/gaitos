//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_MODEL__ROBOTMODEL_HPP_
#define GAITOS_MODEL__ROBOTMODEL_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gaitos_common/Params.hpp"
#include "gaitos_model/Kinematics.hpp"

namespace gaitos
{

/**
 * @struct JointInfo
 * @brief Actuated joint of the robot.
 */
struct JointInfo
{
  std::string name;
  double lower{-1e9};      ///< [rad]
  double upper{1e9};       ///< [rad]
  double effort{0.0};      ///< Max torque [Nm], 0: unknown
  double velocity{0.0};    ///< Max speed [rad/s], 0: unknown
};

/**
 * @struct LegInfo
 * @brief A leg: the chain from the base link to a foot (defined as an SRDF chain group).
 */
struct LegInfo
{
  std::string name;              ///< e.g. FR, left
  std::string group;             ///< SRDF group
  std::string foot;              ///< Tip link of the chain
  bool orientation_ik{false};    ///< Feet with orientation (humanoids): 6D inverse kinematics
  KinematicChain chain;
};

/**
 * @struct SrdfGroup
 * @brief Planning group of the SRDF, resolved to joint names.
 */
struct SrdfGroup
{
  std::string name;
  std::vector<std::string> joints;
  std::vector<std::string> links;
  std::string chain_base;
  std::string chain_tip;
};

/**
 * @struct SrdfData
 * @brief What gaitos uses from an SRDF.
 */
struct SrdfData
{
  std::string robot_name;
  std::map<std::string, SrdfGroup> groups;
  /// Named poses (group_state): name -> joint -> value. States with the same name in
  /// different groups are merged into a single whole-body pose.
  std::map<std::string, std::map<std::string, double>> poses;
  std::map<std::string, std::string> end_effectors;   ///< name -> parent link
};

/**
 * @brief Parse an SRDF
 * @throws std::runtime_error on errors
 */
SrdfData parse_srdf_file(const std::string & file, const KinematicTree & tree);
SrdfData parse_srdf_string(const std::string & xml, const KinematicTree & tree);

/**
 * @brief Parse the kinematic tree of a URDF
 * @throws std::runtime_error on errors
 */
KinematicTree parse_urdf_file(const std::string & file);
KinematicTree parse_urdf_string(const std::string & xml);

/**
 * @class RobotModel
 * @brief Description of a legged robot: actuated joints (in hardware order), legs, named
 * poses and kinematics. Built from the ROS 2 parameters of the node:
 *
 * @code{.yaml}
 * robot_description: "<robot ...>"            # URDF (as robot_state_publisher), or
 * robot_description_semantic: "<robot ...>"   # SRDF (as MoveIt)
 * robot:
 *   name: go2
 *   urdf: package://gaitos_description/robots/go2_description/urdf/go2.urdf   # if no
 *   srdf: package://gaitos_description/robots/go2_description/srdf/go2.srdf   # descriptions
 *   base_link: base            # default: root of the URDF
 *   joints: [FR_hip_joint, ...]  # order of the joint vectors; default: SRDF group "all"
 *   legs:                      # order of the foot vectors
 *     names: [FR, FL, RR, RL]
 *     groups: [FR_leg, FL_leg, RR_leg, RL_leg]   # SRDF chain groups
 *     orientation_ik: [false, false, false, false]   # 6D inverse kinematics (humanoids)
 * @endcode
 */
class RobotModel
{
public:
  /**
   * @brief Build from the node parameters (root block)
   * @throws std::runtime_error / ParamsError on errors
   */
  static std::shared_ptr<RobotModel> from_params(const Params & root);

  /**
   * @brief Build from already parsed data (tests, hosts that have the URDF in memory)
   */
  static std::shared_ptr<RobotModel> create(
    const std::string & name, KinematicTree tree, SrdfData srdf, const Params & robot);

  const std::string & name() const {return name_;}
  const std::string & base_link() const {return base_link_;}

  std::size_t joint_count() const {return joints_.size();}
  const std::vector<JointInfo> & joints() const {return joints_;}
  const JointInfo & joint(std::size_t i) const {return joints_.at(i);}

  /**
   * @brief Index of a joint by name, nullopt if it is not actuated
   */
  std::optional<std::size_t> joint_index(const std::string & name) const;

  /**
   * @brief Indices of the joints of an SRDF group, or of a list of joint names
   * @throws std::runtime_error if a joint is unknown
   */
  std::vector<std::size_t> group_indices(const std::string & group) const;
  std::vector<std::size_t> indices(const std::vector<std::string> & names) const;

  std::size_t leg_count() const {return legs_.size();}
  const std::vector<LegInfo> & legs() const {return legs_;}
  const LegInfo & leg(std::size_t i) const {return legs_.at(i);}

  /**
   * @brief Joints that belong to no leg (arms, waist, head)
   */
  const std::vector<std::size_t> & upper_body_joints() const {return upper_body_;}

  bool has_pose(const std::string & name) const {return srdf_.poses.count(name) > 0;}
  std::vector<std::string> pose_names() const;

  /**
   * @brief Joint vector of a named pose; joints not in the pose take the values of `base`
   * (or 0 if base is empty)
   * @throws std::runtime_error if the pose does not exist
   */
  std::vector<double> pose(const std::string & name, const std::vector<double> & base = {}) const;

  const KinematicTree & tree() const {return tree_;}
  const SrdfData & srdf() const {return srdf_;}

  /**
   * @brief Total mass [kg]
   */
  double mass() const {return mass_;}

  /**
   * @brief Clamp a joint vector to the joint limits
   */
  void clamp_to_limits(std::vector<double> & q) const;

private:
  std::string name_;
  std::string base_link_;
  KinematicTree tree_;
  SrdfData srdf_;
  std::vector<JointInfo> joints_;
  std::map<std::string, std::size_t> joint_map_;
  std::vector<LegInfo> legs_;
  std::vector<std::size_t> upper_body_;
  double mass_{0.0};
};

using RobotModelPtr = std::shared_ptr<const RobotModel>;

}  // namespace gaitos

#endif  // GAITOS_MODEL__ROBOTMODEL_HPP_
