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
