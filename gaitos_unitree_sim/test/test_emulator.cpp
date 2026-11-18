//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "gaitos_system/Gaitos.hpp"
#include "gaitos_system/testing.hpp"
#include "gaitos_unitree_sim/UnitreeGoEmulator.hpp"
#include "gaitos_unitree_sim/UnitreeHgEmulator.hpp"
#include "nlohmann/json.hpp"

using gaitos::Gaitos;
using gaitos_unitree_sim::Reply;
using gaitos_unitree_sim::UnitreeGoEmulator;
using gaitos_unitree_sim::UnitreeHgEmulator;
namespace hg = gaitos_unitree_sim::hg;
namespace go2 = gaitos_unitree_sim::go2;
namespace testutil = gaitos::testutil;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new testutil::RosEnvironment);

namespace
{

/// A robot without physics behind an emulator, its cycles in a thread (the RPC handlers wait
/// for the control loop like on the robot)
template<typename Emulator>
class RobotTest : public ::testing::Test
{
protected:
  void start(const std::string & robot, const std::string & backend, const std::string & extra)
  {
    const std::string src = GAITOS_SOURCE_DIR;
    auto files = testutil::robot_files(src, robot, {backend,
        "gaitos_unitree_sim/params/" + robot + "_unitree_sim.yaml"});
    files.push_back(testutil::overrides(extra));
    const auto root = testutil::make_params(files);
    core.configure(root);
    ASSERT_TRUE(core.start());
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(robot + "_emulator_test");
    emulator.configure(*node, core, root.sub("extensions").sub("unitree"));
    running = true;
    loop = std::thread([this]() {
        while (running) {
          core.step();
          std::this_thread::sleep_for(std::chrono::microseconds(200));
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

  /// Wait (up to 5 s) until gaitos is in `mode`
  bool wait_mode(const std::string & mode)
  {
    for (int i = 0; i < 500; ++i) {
      if (core.snapshot().mode == mode) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  Gaitos core;
  Emulator emulator;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::atomic<bool> running{false};
  std::thread loop;
};

/// R1 (unitree_hg) emulator
class EmulatorTest : public RobotTest<UnitreeHgEmulator>
{
protected:
  void SetUp() override
  {
    start("r1", "gaitos/params/fake.yaml", "hardware: {base_height: 0.75}\ninitial_mode: damp");
  }

  Reply set_fsm(int fsm)
  {
    return emulator.handle_sport(hg::kSetFsmId, "{\"data\": " + std::to_string(fsm) + "}");
  }

  int get_fsm()
  {
    const auto r = emulator.handle_sport(hg::kGetFsmId, "");
    EXPECT_EQ(r.code, 0);
    return nlohmann::json::parse(r.data).at("data").get<int>();
  }
};

/// Go2 (unitree_go) emulator
class Go2EmulatorTest : public RobotTest<UnitreeGoEmulator>
{
protected:
  void SetUp() override
  {
    start("go2", "gaitos/params/go2_fake.yaml", "initial_mode: damp");
  }

  Reply call(int64_t id, const std::string & parameter = "")
  {
    return emulator.handle_sport(id, parameter);
  }
};

}  // namespace

TEST_F(EmulatorTest, FsmMapsToModes)
{
  EXPECT_EQ(emulator.mode_of_fsm(811).value(), "balance_stand");
  EXPECT_EQ(emulator.mode_of_fsm(4).value(), "stand_up");
  EXPECT_FALSE(emulator.mode_of_fsm(2).has_value());
  EXPECT_EQ(emulator.fsm_of_mode("locomotion_lipm").value(), 811);
  EXPECT_EQ(emulator.fsm_of_mode("damp").value(), 1);

  ASSERT_TRUE(wait_mode("damp"));
  EXPECT_EQ(get_fsm(), 1);
  EXPECT_EQ(set_fsm(4).code, 0);
  ASSERT_TRUE(wait_mode("stand_up"));
  EXPECT_EQ(get_fsm(), 4);
  EXPECT_EQ(set_fsm(811).code, 0);
  ASSERT_TRUE(wait_mode("balance_stand"));
  EXPECT_EQ(get_fsm(), 811);
  EXPECT_EQ(set_fsm(811).code, 0);   // already there
  EXPECT_EQ(set_fsm(1).code, 0);
  ASSERT_TRUE(wait_mode("damp"));
}

TEST_F(EmulatorTest, RejectsWhatTheRobotWould)
{
  ASSERT_TRUE(wait_mode("damp"));
  // Walking FSM straight from damp: gaitos refuses balance_stand from damp
  EXPECT_EQ(set_fsm(811).code, gaitos_unitree_sim::code::kRejected);
  EXPECT_EQ(set_fsm(2).code, gaitos_unitree_sim::code::kBadParameter);
  EXPECT_EQ(emulator.handle_sport(hg::kSetFsmId, "not json").code,
    gaitos_unitree_sim::code::kBadParameter);
  // Velocity outside the walking FSM
  EXPECT_EQ(emulator.handle_sport(hg::kSetVelocity,
    R"({"velocity": [0.2, 0.0, 0.0], "duration": 0.5})").code, gaitos_unitree_sim::code::kRejected);
  EXPECT_EQ(emulator.handle_sport(9999, "").code, gaitos_unitree_sim::code::kNotImplemented);
  EXPECT_EQ(emulator.handle_sport(hg::kSetSpeedMode, R"({"data": 1})").code, 0);
}

TEST_F(EmulatorTest, VelocityStartsWalking)
{
  ASSERT_EQ(set_fsm(4).code, 0);
  ASSERT_TRUE(wait_mode("stand_up"));
  ASSERT_EQ(set_fsm(811).code, 0);
  ASSERT_TRUE(wait_mode("balance_stand"));
  EXPECT_EQ(emulator.handle_sport(hg::kSetVelocity,
    R"({"velocity": [0.1, 0.0, 0.0], "duration": 0.5})").code, 0);
  ASSERT_TRUE(wait_mode("locomotion_lipm"));   // on_twist of balance_stand
  EXPECT_EQ(get_fsm(), 811);
}

TEST_F(EmulatorTest, ArmServiceAnswers)
{
  const auto list = emulator.handle_arm(hg::kGetActionList, "");
  EXPECT_EQ(list.code, 0);
  const auto js = nlohmann::json::parse(list.data);
  ASSERT_TRUE(js.is_array());
  EXPECT_EQ(js.size(), 2u);
  EXPECT_EQ(emulator.handle_arm(hg::kStopCustomAction, "").code, 0);
  EXPECT_EQ(emulator.handle_arm(hg::kExecuteAction, R"({"action_id": 1})").code,
    gaitos_unitree_sim::code::kNotImplemented);
}

TEST_F(EmulatorTest, LowStateInRobotOrder)
{
  ASSERT_TRUE(wait_mode("damp"));
  const auto s = core.snapshot();
  unitree_hg::msg::LowState low;
  emulator.fill_lowstate(s, low);
  const auto & model = *core.model();
  for (std::size_t i = 0; i < model.joint_count(); ++i) {
    if (model.joint(i).name == "head_yaw_joint") {
      EXPECT_FLOAT_EQ(low.motor_state[30].q, static_cast<float>(s.joints.position[i]));
    }
    if (model.joint(i).name == "left_hip_pitch_joint") {
      EXPECT_FLOAT_EQ(low.motor_state[0].q, static_cast<float>(s.joints.position[i]));
    }
  }
  EXPECT_EQ(low.motor_state[0].mode, 1);
  EXPECT_EQ(low.motor_state[14].mode, 0);   // empty slot of the 26 DoF R1
  EXPECT_EQ(low.motor_state[30].mode, 1);
  // Upright IMU: quaternion (w, x, y, z) close to identity
  EXPECT_NEAR(std::abs(low.imu_state.quaternion[0]), 1.0, 1e-3);

  unitree_go::msg::SportModeState odom;
  ASSERT_TRUE(emulator.fill_odometry(s, odom));
  EXPECT_NEAR(odom.position[2], s.base.position.z(), 1e-5);
  EXPECT_EQ(odom.mode, 0);   // damp is not the walking FSM
}

TEST_F(Go2EmulatorTest, SportApiDrivesTheModes)
{
  ASSERT_TRUE(wait_mode("damp"));
  EXPECT_EQ(emulator.sport_mode_of("damp").value(), 7);
  // Move outside the walking modes, malformed, unknown api
  EXPECT_EQ(call(go2::kMove, R"({"x": 0.3, "y": 0.0, "z": 0.0})").code,
    gaitos_unitree_sim::code::kRejected);
  EXPECT_EQ(call(go2::kMove, R"({"vx": 0.3})").code, gaitos_unitree_sim::code::kBadParameter);
  EXPECT_EQ(call(1016).code, gaitos_unitree_sim::code::kNotImplemented);   // Hello

  EXPECT_EQ(call(go2::kStandUp).code, 0);
  ASSERT_TRUE(wait_mode("stand_up"));
  EXPECT_EQ(call(go2::kBalanceStand).code, 0);
  ASSERT_TRUE(wait_mode("balance_stand"));
  EXPECT_EQ(call(go2::kEuler, R"({"x": 0.0, "y": 0.1, "z": 0.0})").code, 0);
  EXPECT_EQ(call(go2::kSpeedLevel, R"({"data": 1})").code, 0);
  EXPECT_EQ(call(go2::kMove, R"({"x": 0.3, "y": 0.0, "z": 0.0})").code, 0);
  ASSERT_TRUE(wait_mode("locomotion"));   // on_twist of balance_stand
  EXPECT_EQ(call(go2::kEuler, R"({"x": 0.0, "y": 0.1, "z": 0.0})").code,
    gaitos_unitree_sim::code::kRejected);   // walking: not a posture
  EXPECT_EQ(call(go2::kStopMove).code, 0);
  EXPECT_EQ(call(go2::kBalanceStand).code, 0);
  ASSERT_TRUE(wait_mode("balance_stand"));
  EXPECT_EQ(call(go2::kSit).code, 0);
  ASSERT_TRUE(wait_mode("sit"));
  EXPECT_EQ(call(go2::kRiseSit).code, 0);
  ASSERT_TRUE(wait_mode("balance_stand"));
  EXPECT_EQ(call(go2::kDamp).code, 0);
  ASSERT_TRUE(wait_mode("damp"));
}

TEST_F(Go2EmulatorTest, StateInRobotOrder)
{
  ASSERT_TRUE(wait_mode("damp"));
  const auto s = core.snapshot();
  unitree_go::msg::LowState low;
  emulator.fill_lowstate(s, low);
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_EQ(low.motor_state[i].mode, 1);
    EXPECT_FLOAT_EQ(low.motor_state[i].q, static_cast<float>(s.joints.position[i]));
  }
  EXPECT_EQ(low.motor_state[12].mode, 0);
  EXPECT_EQ(low.head[0], 0xFE);

  unitree_go::msg::SportModeState st;
  ASSERT_TRUE(emulator.fill_sport_state(s, st));
  EXPECT_EQ(st.mode, 7);   // damping
  // FR foot: ahead and to the right of the body, below it; RL: behind and to the left
  EXPECT_GT(st.foot_position_body[0], 0.0f);
  EXPECT_LT(st.foot_position_body[1], 0.0f);
  EXPECT_LT(st.foot_position_body[2], 0.0f);
  EXPECT_LT(st.foot_position_body[9], 0.0f);
  EXPECT_GT(st.foot_position_body[10], 0.0f);
}
