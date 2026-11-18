//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include "gaitos_common/Math.hpp"
#include "gaitos_model/RobotModel.hpp"
#include "gaitos_common/testing.hpp"

using namespace gaitos;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new testutil::RosEnvironment);

namespace
{
const std::string kRobots = std::string(GAITOS_SOURCE_DIR) + "/gaitos_description/robots/";

std::shared_ptr<RobotModel> load(const std::string & robot, const std::string & base,
  const std::vector<std::string> & names, const std::vector<std::string> & groups,
  const std::vector<bool> & orientation = {})
{
  const auto dir = kRobots + robot + "_description/";
  std::vector<rclcpp::Parameter> p{
    {"robot_description", read_file(dir + "urdf/" + robot + ".urdf")},
    {"robot_description_semantic", read_file(dir + "srdf/" + robot + ".srdf")},
    {"robot.base_link", base}, {"robot.legs.names", names}, {"robot.legs.groups", groups}};
  if (!orientation.empty()) {
    p.emplace_back("robot.legs.orientation_ik", orientation);
  }
  return RobotModel::from_params(testutil::make_params({}, p));
}

std::shared_ptr<RobotModel> go2()
{
  return load("go2", "base", {"FR", "FL", "RR", "RL"}, {"FR_leg", "FL_leg", "RR_leg", "RL_leg"});
}
}  // namespace

TEST(Model, Go2)
{
  const auto m = go2();
  ASSERT_EQ(m->joint_count(), 12u);
  ASSERT_EQ(m->leg_count(), 4u);
  // SRDF "all" group order = Unitree motor order
  EXPECT_EQ(m->joint(0).name, "FR_hip_joint");
  EXPECT_EQ(m->joint(11).name, "RL_calf_joint");
  EXPECT_EQ(m->leg(1).foot, "FL_foot");
  EXPECT_EQ(m->leg(0).chain.dof(), 3u);
  EXPECT_TRUE(m->upper_body_joints().empty());
  EXPECT_NEAR(m->mass(), 16.1, 0.5);
  EXPECT_TRUE(m->has_pose("stand"));
  EXPECT_TRUE(m->has_pose("lie"));
  const auto q = m->pose("stand");
  EXPECT_DOUBLE_EQ(q[1], 0.67);
  EXPECT_DOUBLE_EQ(q[2], -1.3);
  // Feet below the base, front feet ahead, right feet on the right
  const auto fr = m->leg(0).chain.forward(q).translation();
  const auto rl = m->leg(3).chain.forward(q).translation();
  EXPECT_LT(fr.z(), -0.25);
  EXPECT_GT(fr.x(), 0.1);
  EXPECT_LT(fr.y(), 0.0);
  EXPECT_LT(rl.x(), -0.1);
  EXPECT_GT(rl.y(), 0.0);
}

TEST(Model, JacobianMatchesFiniteDifferences)
{
  const auto m = go2();
  auto q = m->pose("stand");
  q[3] = 0.2;
  const auto & chain = m->leg(1).chain;
  const Eigen::MatrixXd j = chain.jacobian(q);
  const double h = 1e-6;
  for (std::size_t k = 0; k < chain.dof(); ++k) {
    auto qp = q;
    qp[static_cast<std::size_t>(chain.joint_indices()[k])] += h;
    const Eigen::Vector3d d = (chain.forward(qp).translation() - chain.forward(q).translation()) / h;
    EXPECT_TRUE(d.isApprox(j.block<3, 1>(0, static_cast<Eigen::Index>(k)), 1e-4)) << k;
  }
}

TEST(Model, InverseKinematics)
{
  const auto m = go2();
  const auto stand = m->pose("stand");
  for (std::size_t l = 0; l < m->leg_count(); ++l) {
    const auto & chain = m->leg(l).chain;
    Eigen::Isometry3d target = chain.forward(stand);
    target.translation() += Eigen::Vector3d(0.04, -0.02, 0.05);
    auto q = stand;
    const double err = chain.inverse(target, q, {});
    EXPECT_LT(err, 1e-4);
    EXPECT_TRUE(chain.forward(q).translation().isApprox(target.translation(), 1e-3));
  }
}

TEST(Model, R1HumanoidPoseIk)
{
  const auto m = load("r1", "pelvis_link", {"left", "right"}, {"left_leg", "right_leg"}, {true, true});
  ASSERT_EQ(m->joint_count(), 26u);
  EXPECT_EQ(m->joint(0).name, "left_hip_pitch_joint");
  EXPECT_EQ(m->joint(12).name, "waist_roll_joint");
  EXPECT_EQ(m->joint(25).name, "head_yaw_joint");
  EXPECT_EQ(m->leg(0).chain.dof(), 6u);
  EXPECT_EQ(m->upper_body_joints().size(), 14u);
  const auto stand = m->pose("stand");
  const auto & chain = m->leg(0).chain;
  Eigen::Isometry3d target = chain.forward(stand);
  target.translation().z() += 0.04;
  auto q = stand;
  gaitos::KinematicChain::IkOptions o;
  o.orientation = true;
  o.max_iterations = 60;
  EXPECT_LT(chain.inverse(target, q, o), 1e-3);
  EXPECT_LT(rotation_error(target.linear(), chain.forward(q).linear()).norm(), 1e-2);
}

TEST(Model, NaoWithMimicJoint)
{
  const auto m = load("nao", "torso", {"left", "right"}, {"left_leg", "right_leg"});
  EXPECT_FALSE(m->joint_index("RHipYawPitch").has_value());   // mimic of LHipYawPitch
  ASSERT_TRUE(m->joint_index("LHipYawPitch").has_value());
  // The right leg chain is driven by LHipYawPitch
  const auto & idx = m->leg(1).chain.joint_indices();
  EXPECT_EQ(static_cast<std::size_t>(idx.front()), *m->joint_index("LHipYawPitch"));
  EXPECT_TRUE(m->has_pose("crouch"));
}

TEST(Model, Errors)
{
  EXPECT_THROW(RobotModel::from_params(testutil::make_params({}, {{"robot.urdf", "/nonexistent.urdf"}})),
    std::runtime_error);
  EXPECT_THROW(go2()->pose("nope"), std::runtime_error);
}
