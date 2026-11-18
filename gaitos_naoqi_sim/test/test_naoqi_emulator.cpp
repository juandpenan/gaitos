//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "gaitos_naoqi_sim/NaoqiDriverEmulator.hpp"
#include "gaitos_system/Gaitos.hpp"
#include "gaitos_system/testing.hpp"

using gaitos::Gaitos;
using gaitos_naoqi_sim::NaoqiDriverEmulator;
namespace testutil = gaitos::testutil;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new testutil::RosEnvironment);

namespace
{

/// NAO without physics behind the emulator, its cycles in a thread (the commands wait for the
/// control loop like with NAOqi)
class NaoqiTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string src = GAITOS_SOURCE_DIR;
    const auto root = testutil::make_params(testutil::robot_files(src, "nao",
        {"gaitos/params/nao_fake.yaml", "gaitos_naoqi_sim/params/nao_naoqi_sim.yaml"}));
    core.configure(root);
    ASSERT_TRUE(core.start());
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("naoqi_emulator_test");
    emulator.configure(*node, core, root.sub("extensions").sub("naoqi"));
    running = true;
    loop = std::thread([this]() {
        while (running) {
          core.step();
          std::this_thread::sleep_for(std::chrono::microseconds(300));
        }
      });
  }

  void TearDown() override
  {
    running = false;
    if (loop.joinable()) {
      loop.join();
    }
    core.stop();
  }

  bool wait(const std::function<bool()> & cond, double seconds = 5.0)
  {
    for (int i = 0; i < static_cast<int>(seconds * 100); ++i) {
      if (cond()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  bool wait_mode(const std::string & mode)
  {
    return wait([&]() {return core.snapshot().mode == mode;});
  }

  void request(const std::string & mode)
  {
    auto f = core.request_mode(mode);
    ASSERT_EQ(f.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    ASSERT_TRUE(f.get().success);
  }

  double joint(const std::string & name)
  {
    return core.snapshot().joints.position[*core.model()->joint_index(name)];
  }

  Gaitos core;
  NaoqiDriverEmulator emulator;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::atomic<bool> running{false};
  std::thread loop;
};

naoqi_bridge_msgs::msg::JointAnglesWithSpeed angles(
  std::vector<std::string> names, std::vector<float> values, float speed, bool relative = false)
{
  naoqi_bridge_msgs::msg::JointAnglesWithSpeed m;
  m.joint_names = std::move(names);
  m.joint_angles = std::move(values);
  m.speed = speed;
  m.relative = relative ? 1 : 0;
  return m;
}

}  // namespace

TEST_F(NaoqiTest, JointAnglesNamesAndChains)
{
  const auto head = emulator.joint_targets(angles({"Head"}, {0.3f, -0.2f}, 0.5f));
  ASSERT_EQ(head.size(), 2u);
  EXPECT_EQ(head[0].name, "HeadYaw");
  EXPECT_EQ(head[1].name, "HeadPitch");
  const double vmax = core.model()->joint(*core.model()->joint_index("HeadYaw")).velocity;
  EXPECT_NEAR(head[0].speed, 0.5 * vmax, 1e-6);
  // Body: 26 angles; RHipYawPitch follows LHipYawPitch, as on the robot
  const auto body = emulator.joint_targets(angles({"Body"}, std::vector<float>(26, 0.0f), 0.1f));
  EXPECT_EQ(body.size(), 25u);
  // Names and angles that do not match: nothing
  EXPECT_TRUE(emulator.joint_targets(angles({"Head"}, {0.3f}, 0.5f)).empty());
  EXPECT_TRUE(emulator.joint_targets(angles({"HeadYaw", "HeadPitch"}, {0.3f}, 0.5f)).empty());
  // Relative
  const auto rel = emulator.joint_targets(angles({"HeadYaw"}, {0.1f}, 0.2f, true));
  ASSERT_EQ(rel.size(), 1u);
  EXPECT_TRUE(rel[0].relative);
}

TEST_F(NaoqiTest, JointAnglesMoveTheHeadOverTheStance)
{
  request("crouch");
  ASSERT_TRUE(wait([&]() {return core.state().cycle > 300;}));
  request("stand_init");
  ASSERT_TRUE(wait([&]() {return core.state().cycle > 300;}));
  request("balance_stand");
  ASSERT_TRUE(wait_mode("balance_stand"));
  core.set_joint_targets(emulator.joint_targets(angles({"HeadYaw"}, {0.5f}, 0.5f)));
  EXPECT_TRUE(wait([&]() {return std::abs(joint("HeadYaw") - 0.5) < 0.02;}));
  core.set_joint_targets(emulator.joint_targets(angles({"HeadYaw"}, {-0.2f}, 0.5f, true)));
  EXPECT_TRUE(wait([&]() {return std::abs(joint("HeadYaw") - 0.3) < 0.02;}));
  // A leg while the stance controller holds the legs: not allowed
  const double knee = joint("LKneePitch");
  core.set_joint_targets(emulator.joint_targets(angles({"LKneePitch"}, {1.5f}, 1.0f)));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_NEAR(joint("LKneePitch"), knee, 0.05);
  // A new mode takes the head back
  request("stand_init");
  EXPECT_TRUE(wait([&]() {return std::abs(joint("HeadYaw")) < 0.02;}));
}

TEST_F(NaoqiTest, MoveOnlyWhileStanding)
{
  request("crouch");
  ASSERT_TRUE(wait([&]() {return core.state().cycle > 300;}));
  EXPECT_FALSE(emulator.move(0.05, 0.0, 0.0)) << "a resting NAO does not walk";
  EXPECT_TRUE(emulator.move(0.0, 0.0, 0.0)) << "stopping is always fine";
  request("stand_init");
  ASSERT_TRUE(wait([&]() {return core.state().cycle > 300;}));
  EXPECT_TRUE(emulator.move(0.05, 0.0, 0.0));
  EXPECT_TRUE(wait_mode("locomotion")) << "stand_init -> balance_stand -> walking";
  EXPECT_TRUE(emulator.move(0.0, 0.0, 0.0));
}

TEST_F(NaoqiTest, MoveToStartsTheWalk)
{
  request("stand_init");
  ASSERT_TRUE(wait([&]() {return core.state().cycle > 300;}));
  EXPECT_TRUE(emulator.move_to(0.2, 0.0, 0.0));
  EXPECT_TRUE(wait_mode("locomotion"));
}

TEST_F(NaoqiTest, JointStatesOfTheWholeBody)
{
  ASSERT_TRUE(wait([&]() {return core.snapshot().cycle > 10;}));
  const auto s = core.snapshot();
  const auto js = emulator.joint_state(s);
  ASSERT_EQ(js.name.size(), 26u);
  EXPECT_EQ(js.name.front(), "HeadYaw");
  EXPECT_EQ(js.name.back(), "RHand");
  const auto l = std::find(js.name.begin(), js.name.end(), "LHipYawPitch") - js.name.begin();
  const auto r = std::find(js.name.begin(), js.name.end(), "RHipYawPitch") - js.name.begin();
  ASSERT_LT(static_cast<std::size_t>(r), js.name.size());
  EXPECT_DOUBLE_EQ(js.position[static_cast<std::size_t>(l)], js.position[static_cast<std::size_t>(r)]);
}
