//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "gaitos_hardware/FakeHardware.hpp"
#include "gaitos_system/Gaitos.hpp"
#include "gaitos_system/Safety.hpp"
#include "gaitos_system/testing.hpp"

using namespace gaitos;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new gaitos::testutil::RosEnvironment);

namespace
{
/// go2 on fake hardware plus overrides
Params config(const std::string & yaml)
{
  auto f = testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos/params/go2_fake.yaml"});
  f.push_back(testutil::overrides(yaml));
  return testutil::make_params(f);
}

gaitos_hardware::FakeHardware & fake(Gaitos & g)
{
  return dynamic_cast<gaitos_hardware::FakeHardware &>(g.hardware());
}

void run(Gaitos & g, double seconds)
{
  for (int i = 0; i < static_cast<int>(seconds * g.frequency()); ++i) {
    g.step();
  }
}

RemoteInput press(uint32_t buttons, double stamp = 0.0)
{
  RemoteInput r;
  r.valid = true;
  r.buttons = buttons;
  r.stamp = stamp;
  return r;
}
}  // namespace

TEST(Safety, ParseButtons)
{
  EXPECT_EQ(parse_buttons("L2+B"), RemoteButton::L2 | RemoteButton::B);
  EXPECT_EQ(parse_buttons("start"), RemoteButton::START);
  EXPECT_EQ(parse_buttons("L2 + nope"), 0u);
}

TEST(Safety, MonitorTripsAfterConsecutiveCycles)
{
  const auto model = RobotModel::from_params(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos/params/go2_fake.yaml"})));
  SafetyMonitor m;
  m.configure(testutil::make_params({testutil::overrides("cycles: 2")}), model);
  JointState j;
  j.resize(12);
  j.position = model->pose("stand");
  EXPECT_TRUE(m.check(j).empty());
  j.velocity[1] = 100.0;          // thigh limit 30.1 rad/s
  EXPECT_TRUE(m.check(j).empty());   // first cycle: not yet
  const auto why = m.check(j);
  EXPECT_NE(why.find("FR_thigh_joint"), std::string::npos) << why;
  EXPECT_NE(why.find("velocity"), std::string::npos);
  j.velocity[1] = 0.0;
  j.effort[2] = 80.0;             // calf limit 45.43 Nm
  m.check(j);
  EXPECT_NE(m.check(j).find("torque"), std::string::npos);
  j.effort[2] = 0.0;
  j.position[2] = 0.5;            // calf range [-2.72, -0.84]
  m.check(j);
  EXPECT_NE(m.check(j).find("position"), std::string::npos);
}

TEST(Safety, TeleopEdgesAndSticks)
{
  Teleop t;
  t.configure(testutil::make_params({testutil::overrides("enabled: true\nestop: L2+B\nbuttons: [L2+A, A]\nmodes: [stand_up, sit]\n"
      "twist_scale: [0.5, 0.3, 1.0]\ndeadzone: 0.1\ntimeout: 0.5")}));
  auto a = t.update(press(RemoteButton::L2 | RemoteButton::A, 0.0), 0.0);
  EXPECT_EQ(a.mode, "stand_up");   // the longer combination wins
  a = t.update(press(RemoteButton::L2 | RemoteButton::A, 0.01), 0.01);
  EXPECT_TRUE(a.mode.empty());     // only on press
  a = t.update(press(RemoteButton::L2 | RemoteButton::B, 0.02), 0.02);
  EXPECT_TRUE(a.estop);
  RemoteInput r = press(0, 0.03);
  r.left = Eigen::Vector2d(0.0, 1.0);
  r.right = Eigen::Vector2d(0.5, 0.0);
  a = t.update(r, 0.03);
  ASSERT_TRUE(a.twist);
  EXPECT_DOUBLE_EQ(a.linear.x(), 0.5);
  EXPECT_DOUBLE_EQ(a.angular.z(), -0.5);
  a = t.update(press(0, 0.04), 0.04);    // released: one zero twist
  EXPECT_TRUE(a.twist);
  EXPECT_DOUBLE_EQ(a.linear.norm(), 0.0);
  a = t.update(press(0, 0.05), 0.05);
  EXPECT_FALSE(a.twist);
  a = t.update(r, 1.0);            // stale remote (stamp 0.03): ignored
  EXPECT_FALSE(a.twist);
}

TEST(Safety, DryRunNeverWrites)
{
  Gaitos g;
  g.configure(config("dry_run: true"));
  ASSERT_TRUE(g.start());
  auto f = g.request_mode("stand_up");
  run(g, 1.0);
  EXPECT_TRUE(f.get().success);
  EXPECT_EQ(g.state().mode, "stand_up");   // everything runs...
  EXPECT_EQ(fake(g).writes(), 0u);         // ...but nothing reaches the hardware
  EXPECT_TRUE(g.snapshot().dry_run);
}

TEST(Safety, RemoteModesAndEmergencyStop)
{
  Gaitos g;
  g.configure(config("teleop: {enabled: true, estop: L2+B, buttons: [L2+A], modes: [stand_up]}"));
  ASSERT_TRUE(g.start());
  run(g, 0.1);
  fake(g).set_remote(press(RemoteButton::L2 | RemoteButton::A));
  run(g, 0.1);
  EXPECT_EQ(g.state().mode, "stand_up");
  fake(g).set_remote(press(0));
  run(g, 1.0);
  fake(g).set_remote(press(RemoteButton::L2 | RemoteButton::B));
  g.step();
  EXPECT_EQ(g.state().mode, "damp");
  EXPECT_NE(g.snapshot().last_safety_event.find("emergency stop"), std::string::npos);
}

TEST(Safety, GainRampFromPassiveModes)
{
  Gaitos g;
  g.configure(config("safety: {gain_ramp_time: 1.0}"));
  ASSERT_TRUE(g.start());
  run(g, 0.1);
  auto f = g.request_mode("stand_up");
  g.step();
  ASSERT_TRUE(f.get().success);
  const double kp_start = g.state().motor_command.kp[1];
  run(g, 0.5);
  const double kp_half = g.state().motor_command.kp[1];
  run(g, 0.6);
  const double kp_full = g.state().motor_command.kp[1];
  EXPECT_LT(kp_start, 0.05 * kp_full);
  EXPECT_NEAR(kp_half, 0.5 * kp_full, 0.05 * kp_full);
  EXPECT_DOUBLE_EQ(kp_full, 60.0);
}

TEST(Safety, WatchdogStopsTheHardwareWhenTheCycleStalls)
{
  Gaitos g;
  g.configure(config("safety: {watchdog_timeout: 0.05}"));
  ASSERT_TRUE(g.start());
  std::atomic<bool> running{true};
  std::thread loop([&]() {g.run(running);});
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(fake(g).emergency_stops(), 0);
  fake(g).stall(0.3);             // the cycle blocks 300 ms
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  running = false;
  loop.join();
  EXPECT_EQ(fake(g).emergency_stops(), 1);
}
