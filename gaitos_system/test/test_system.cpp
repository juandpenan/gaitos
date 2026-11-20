//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <cmath>

#include "gaitos_core/PluginLoader.hpp"
#include "gaitos_system/testing.hpp"
#include "gaitos_system/Gaitos.hpp"

using namespace gaitos;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new gaitos::testutil::RosEnvironment);

namespace
{

CommandResult request(Gaitos & g, const std::string & mode)
{
  auto f = g.request_mode(mode);
  g.step();
  return f.get();
}

void run(Gaitos & g, double seconds)
{
  const int n = static_cast<int>(seconds * g.frequency());
  for (int i = 0; i < n; ++i) {
    ASSERT_TRUE(g.step());
  }
}

double max_error(const std::vector<double> & a, const std::vector<double> & b)
{
  double e = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    e = std::max(e, std::abs(a[i] - b[i]));
  }
  return e;
}
}  // namespace

TEST(System, PluginsAreDiscovered)
{
  PluginLoader<ControllerBase> loader("gaitos::ControllerBase");
  const auto names = loader.available();
  EXPECT_NE(std::find(names.begin(), names.end(), "gaitos_controllers/GaitController"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "gaitos_dynamics/ConvexMpcController"), names.end());
  const auto params = testutil::make_params({testutil::overrides("plugin: gaitos_controllers/Nope")});
  EXPECT_THROW(loader.create("x", params, nullptr), std::runtime_error);
}

TEST(System, Go2ModesOnFakeHardware)
{
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos/params/go2_fake.yaml"})));
  ASSERT_TRUE(g.start());
  run(g, 0.1);
  EXPECT_EQ(g.state().mode, "damp");
  EXPECT_TRUE(g.state().base.valid);

  // Sit is only allowed while standing
  EXPECT_FALSE(request(g, "sit").success);
  EXPECT_FALSE(request(g, "unknown").success);

  ASSERT_TRUE(request(g, "stand_up").success);
  run(g, 2.0);
  const auto stand = g.model()->pose("stand");
  EXPECT_LT(max_error(g.state().joints.position, stand), 0.05);

  ASSERT_TRUE(request(g, "balance_stand").success);
  run(g, 0.5);
  EXPECT_EQ(g.state().controller, "stance");
  // Lower the body 5 cm: knees bend more
  g.set_pose_goal({0.0, 0.0, -0.05}, Eigen::Quaterniond::Identity());
  run(g, 1.0);
  EXPECT_NEAR(g.state().reference.body_offset.z(), -0.05, 1e-9);
  EXPECT_LT(g.state().command.position[2], stand[2] - 0.05);

  // A twist starts walking (on_twist), stopping goes back to standing still
  g.set_twist({0.3, 0.0, 0.0}, Eigen::Vector3d::Zero());
  g.step();
  g.step();
  EXPECT_EQ(g.state().mode, "locomotion");
  for (int i = 0; i < 500; ++i) {
    g.set_twist({0.3, 0.0, 0.0}, Eigen::Vector3d::Zero());
    ASSERT_TRUE(g.step());
    for (const double q : g.state().motor_command.position) {
      ASSERT_TRUE(std::isfinite(q));
    }
  }
  run(g, 1.5);   // command timeout -> stops

  ASSERT_TRUE(request(g, "stand_down").success);
  run(g, 2.5);
  EXPECT_LT(max_error(g.state().joints.position, g.model()->pose("lie")), 0.08);
  ASSERT_TRUE(request(g, "damp").success);
  g.stop();
}

TEST(System, PostureChainsToNextMode)
{
  std::vector<std::string> changes;   // outlives g: stop() notifies the last change
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos/params/go2_fake.yaml"})));
  ASSERT_TRUE(g.start());
  g.set_mode_callback([&changes](const std::string &, const std::string & to, const std::string &) {
      changes.push_back(to);
    });
  ASSERT_TRUE(request(g, "recovery_stand").success);
  run(g, 3.0);
  EXPECT_EQ(g.state().mode, "balance_stand");
  ASSERT_GE(changes.size(), 2u);
  EXPECT_EQ(changes.back(), "balance_stand");
}

TEST(System, NaoDescriptionRuns)
{
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "nao", {"gaitos/params/nao_fake.yaml"})));
  ASSERT_TRUE(g.start());
  ASSERT_TRUE(request(g, "stand_init").success);
  run(g, 6.0);   // soft servos (stiffness 0.8) on the fake hardware
  EXPECT_LT(max_error(g.state().joints.position, g.model()->pose("stand_init", g.state().joints.position)), 0.05);
  // Position servos: the output is a position with a stiffness
  for (std::size_t i = 0; i < g.model()->joint_count(); ++i) {
    EXPECT_DOUBLE_EQ(g.state().motor_command.kp[i], 0.8);
  }
}

TEST(System, JointCommandOverridesBlendAndRelease)
{
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "nao", {"gaitos/params/nao_fake.yaml"})));
  ASSERT_TRUE(g.start());
  ASSERT_TRUE(request(g, "stand_init").success);
  run(g, 0.5);
  const auto i = *g.model()->joint_index("HeadYaw");
  const double controller_kp = g.state().command.kp[i];   // what the controller commands alone
  const double controller_effort = g.state().command.effort[i];
  ASSERT_NE(controller_kp, 5.0);

  JointCommandOverride c;
  c.name = "HeadYaw";
  c.position = 0.3;
  c.velocity = 0.1;
  c.effort = 0.2;
  c.kp = 5.0;
  c.kd = 0.5;
  c.weight = 1.0;
  JointCommandOverride unknown = c;
  unknown.name = "NoSuchJoint";
  EXPECT_EQ(g.set_joint_commands({c, unknown}), 1u);
  g.step();
  EXPECT_DOUBLE_EQ(g.state().command.position[i], 0.3);
  EXPECT_DOUBLE_EQ(g.state().command.velocity[i], 0.1);
  EXPECT_DOUBLE_EQ(g.state().command.effort[i], 0.2);
  EXPECT_DOUBLE_EQ(g.state().command.kp[i], 5.0);
  EXPECT_DOUBLE_EQ(g.state().command.kd[i], 0.5);

  // Not finite: ignored, the previous command stays
  JointCommandOverride bad = c;
  bad.position = std::nan("");
  bad.kp = 50.0;
  g.set_joint_commands({bad});
  g.step();
  EXPECT_DOUBLE_EQ(g.state().command.kp[i], 5.0);

  // Out of the joint limits: clamped
  JointCommandOverride big = c;
  big.position = 100.0;
  big.effort = 1e6;
  g.set_joint_commands({big});
  g.step();
  EXPECT_DOUBLE_EQ(g.state().command.position[i], g.model()->joint(i).upper);
  EXPECT_LE(g.state().command.effort[i], g.model()->joint(i).effort);

  c.weight = 0.5;   // half the controller, half the override
  g.set_joint_commands({c});
  g.step();
  EXPECT_NEAR(g.state().command.kp[i], 0.5 * controller_kp + 2.5, 1e-9);
  EXPECT_NEAR(g.state().command.effort[i], 0.5 * controller_effort + 0.1, 1e-9);

  // No update for command_timeout (0.5 s): blended out over release_time (0.2 s)
  c.weight = 1.0;
  g.set_joint_commands({c});
  run(g, 0.6);
  const double kp_releasing = g.state().command.kp[i];
  EXPECT_GT(std::abs(kp_releasing - controller_kp), 1e-6);   // not back yet
  run(g, 0.2);
  EXPECT_NEAR(g.state().command.kp[i], controller_kp, 1e-9);  // back to the controller
}

TEST(System, JointCommandOverridesEndOnModeChange)
{
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "nao", {"gaitos/params/nao_fake.yaml"})));
  ASSERT_TRUE(g.start());
  ASSERT_TRUE(request(g, "stand_init").success);
  run(g, 0.5);
  const auto i = *g.model()->joint_index("HeadYaw");
  JointCommandOverride c;
  c.name = "HeadYaw";
  c.kp = 5.0;
  g.set_joint_commands({c});
  g.step();
  EXPECT_DOUBLE_EQ(g.state().command.kp[i], 5.0);
  ASSERT_TRUE(request(g, "damp").success);
  run(g, 0.3);   // more than release_time
  JointCommandOverride later = c;
  later.kp = 7.0;   // in damp (a passive mode for the NAO: its safe mode) commands are refused
  g.set_joint_commands({later});
  g.step();
  EXPECT_NE(g.state().command.kp[i], 5.0);
  EXPECT_NE(g.state().command.kp[i], 7.0);
}

TEST(System, JointCommandTimeoutMustBePositive)
{
  Gaitos g;
  auto files = testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "nao", {"gaitos/params/nao_fake.yaml"});
  files.push_back(testutil::overrides("external_joints: {command_timeout: 0.0}"));
  EXPECT_THROW(g.configure(testutil::make_params(files)), std::runtime_error);
}
