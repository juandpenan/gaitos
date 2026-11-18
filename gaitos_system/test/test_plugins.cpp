//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include "gaitos_common/Math.hpp"
#include "gaitos_core/ControllerBase.hpp"
#include "gaitos_core/JointActuatorBase.hpp"
#include "gaitos_core/PluginLoader.hpp"
#include "gaitos_system/testing.hpp"
#include "gaitos_core/SensorBase.hpp"
#include "gaitos_core/StateEstimatorBase.hpp"
#include "gaitos_system/Gaitos.hpp"

using namespace gaitos;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new gaitos::testutil::RosEnvironment);

namespace
{
const std::string kRobots = std::string(GAITOS_SOURCE_DIR) + "/gaitos_description/robots/";

RobotModelPtr go2()
{
  static RobotModelPtr model = RobotModel::from_params(
    testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos/params/go2_fake.yaml"})));
  return model;
}

template<typename Base> const char * base_name();
template<> const char * base_name<JointActuatorBase>() {return "gaitos::JointActuatorBase";}
template<> const char * base_name<ContactSensorBase>() {return "gaitos::ContactSensorBase";}
template<> const char * base_name<ImuSensorBase>() {return "gaitos::ImuSensorBase";}
template<> const char * base_name<StateEstimatorBase>() {return "gaitos::StateEstimatorBase";}
template<> const char * base_name<ControllerBase>() {return "gaitos::ControllerBase";}

template<typename Base>
std::shared_ptr<Base> make(const std::string & yaml)
{
  static PluginLoader<Base> loader(base_name<Base>());   // outlives the plugins
  return loader.create("test", testutil::make_params({testutil::overrides(yaml)}), go2());
}

RobotState standing_state()
{
  RobotState s;
  s.resize(go2()->joint_count(), go2()->leg_count());
  s.joints.position = go2()->pose("stand");
  s.imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, kGravity);
  s.dt = 0.002;
  std::fill(s.contacts.contact.begin(), s.contacts.contact.end(), true);
  return s;
}
}  // namespace

TEST(Actuators, TorquePdClosesTheLoop)
{
  auto a = make<JointActuatorBase>("plugin: gaitos_actuators/TorquePd\nmax_torque: 5.0");
  a->set_joints({0, 1});
  JointState m;
  m.resize(12);
  JointCommand in, out;
  in.resize(12);
  out.resize(12);
  in.position[0] = 0.1;
  in.kp[0] = 20.0;
  in.position[1] = 1.0;
  in.kp[1] = 100.0;
  a->update(m, in, out);
  EXPECT_NEAR(out.effort[0], 2.0, 1e-9);
  EXPECT_NEAR(out.effort[1], 5.0, 1e-9);   // limited
  EXPECT_EQ(out.kp[0], 0.0);
}

TEST(Actuators, PdPassthroughLimitsTargets)
{
  auto a = make<JointActuatorBase>("plugin: gaitos_actuators/PdPassthrough\nposition_margin: 0.1");
  a->set_joints({2});
  JointState m;
  m.resize(12);
  JointCommand in, out;
  in.resize(12);
  out.resize(12);
  in.position[2] = 0.0;   // calf range [-2.72, -0.84]
  in.kp[2] = 30.0;
  a->update(m, in, out);
  EXPECT_NEAR(out.position[2], go2()->joint(2).upper - 0.1, 1e-9);
  EXPECT_EQ(out.kp[2], 30.0);
}

TEST(Sensors, ForceThresholdHysteresis)
{
  auto c = make<ContactSensorBase>("plugin: gaitos_sensors/ForceThresholdContact\non_threshold: 20\noff_threshold: 10");
  RobotState s = standing_state();
  ContactData out;
  out.resize(4);
  s.foot_force_raw = {25.0, 15.0, 5.0, 0.0};
  c->update(s, out);
  EXPECT_TRUE(out.contact[0]);
  EXPECT_FALSE(out.contact[1]);   // never above on_threshold
  s.foot_force_raw = {15.0, 15.0, 5.0, 0.0};
  c->update(s, out);
  EXPECT_TRUE(out.contact[0]);    // between thresholds: keeps contact
  s.foot_force_raw = {5.0, 15.0, 5.0, 0.0};
  c->update(s, out);
  EXPECT_FALSE(out.contact[0]);
}

TEST(Sensors, TorqueContactFromJointTorques)
{
  auto c = make<ContactSensorBase>("plugin: gaitos_sensors/TorqueContact\nfilter: 0.0");
  RobotState s = standing_state();
  // Torques that hold 40 N on every foot: tau = -J^T F
  for (std::size_t l = 0; l < 4; ++l) {
    const auto & chain = go2()->leg(l).chain;
    const Eigen::VectorXd tau = -chain.jacobian(s.joints.position).topRows(3).transpose() *
      Eigen::Vector3d(0.0, 0.0, l < 2 ? 40.0 : 0.0);
    for (std::size_t k = 0; k < 3; ++k) {
      s.joints.effort[static_cast<std::size_t>(chain.joint_indices()[k])] = tau(static_cast<Eigen::Index>(k));
    }
  }
  ContactData out;
  out.resize(4);
  c->update(s, out);
  EXPECT_NEAR(out.force[0], 40.0, 0.01);   // damped pseudoinverse
  EXPECT_TRUE(out.contact[0]);
  EXPECT_FALSE(out.contact[2]);
}

TEST(Sensors, ComplementaryImuLevelsFromAccelerometer)
{
  auto imu = make<ImuSensorBase>("plugin: gaitos_sensors/ComplementaryImu\nkp: 2.0");
  ImuData raw, out;
  raw.has_orientation = false;
  const Eigen::Quaterniond tilted = rpy_to_quaternion({0.2, -0.1, 0.0});
  raw.linear_acceleration = tilted.conjugate() * Eigen::Vector3d(0.0, 0.0, kGravity);
  for (int i = 0; i < 2000; ++i) {
    imu->update(raw, 0.002, out);
  }
  const Eigen::Vector3d rpy = quaternion_to_rpy(out.orientation);
  EXPECT_NEAR(rpy.x(), 0.2, 0.01);
  EXPECT_NEAR(rpy.y(), -0.1, 0.01);
}

TEST(Estimators, KalmanStandingAndExternalOdometry)
{
  auto est = make<StateEstimatorBase>(
    "plugin: gaitos_estimators/KalmanEstimator\n"
    "external: {mocap: {position: true, yaw: true, yaw_gain: 0.1, align: false, position_noise: 0.01}}");
  RobotState s = standing_state();
  BaseState b;
  for (int i = 0; i < 500; ++i) {
    s.time += s.dt;
    ASSERT_TRUE(est->update(s, b));
  }
  // Standing still: height from the leg kinematics, no drift
  const double height = -go2()->leg(0).chain.forward(s.joints.position).translation().z();
  EXPECT_NEAR(b.position.z(), height, 0.01);
  EXPECT_LT(b.position.head<2>().norm(), 1e-3);
  EXPECT_LT(b.linear_velocity.norm(), 1e-3);

  // An external odometry 0.2 m ahead with a 0.3 rad yaw pulls the estimate
  auto & ext = s.external_odometry["mocap"];
  ext.odometry.position = Eigen::Vector3d(0.2, 0.0, height);
  ext.odometry.orientation = rpy_to_quaternion({0.0, 0.0, 0.3});
  for (int i = 0; i < 400; ++i) {
    s.time += s.dt;
    ext.reception = s.time;
    ++ext.seq;
    est->update(s, b);
  }
  EXPECT_NEAR(yaw_of(b.orientation), 0.3, 0.01);
  EXPECT_GT(b.position.x(), 0.05);
}

TEST(Estimators, ExternalOdometryTimesOut)
{
  auto est = make<StateEstimatorBase>("plugin: gaitos_estimators/ExternalOdometryEstimator\nsource: gt\ntimeout: 0.1");
  RobotState s = standing_state();
  BaseState b;
  EXPECT_FALSE(est->update(s, b));
  auto & ext = s.external_odometry["gt"];
  ext.odometry.position = Eigen::Vector3d(1.0, 2.0, 0.3);
  ext.seq = 1;
  ext.reception = 0.0;
  EXPECT_TRUE(est->update(s, b));
  EXPECT_TRUE(b.valid);
  EXPECT_NEAR(b.position.y(), 2.0, 1e-12);
  s.time = 0.5;
  est->update(s, b);
  EXPECT_FALSE(b.valid);
}

TEST(Controllers, GaitAlternatesDiagonalPairsAndStops)
{
  auto c = (make<ControllerBase>(
      "plugin: gaitos_controllers/GaitController\n"
      "gaits: {trot: {offsets: [0.0, 0.5, 0.5, 0.0], period: 0.4, duty: 0.5, step_height: 0.06}}\n"
      "idle_time: 0.1\n"));
  RobotState s = standing_state();
  c->activate(s);
  ControlReference ref;
  ref.joints.resize(12);
  ref.linear_velocity.x() = 0.3;
  JointCommand out;
  out.resize(12);
  bool fr_rl_swing = false;
  for (int i = 0; i < 400; ++i) {
    ASSERT_TRUE(c->update(s, ref, 0.002, out));
  }
  // Swing legs lift: FR/RL feet higher than FL/RR at some phase
  for (int i = 0; i < 200; ++i) {
    c->update(s, ref, 0.002, out);
    const double fr = go2()->leg(0).chain.forward(out.position).translation().z();
    const double fl = go2()->leg(1).chain.forward(out.position).translation().z();
    const double rl = go2()->leg(3).chain.forward(out.position).translation().z();
    if (fr > fl + 0.03 && std::abs(fr - rl) < 0.005) {
      fr_rl_swing = true;
    }
  }
  EXPECT_TRUE(fr_rl_swing);
  // Zero command: the legs stop at their next touchdown
  ref.linear_velocity.setZero();
  for (int i = 0; i < 500; ++i) {
    c->update(s, ref, 0.002, out);
  }
  for (std::size_t l = 0; l < 4; ++l) {
    const auto p = go2()->leg(l).chain.forward(out.position).translation();
    EXPECT_NEAR(p.z(), go2()->leg(l).chain.forward(go2()->pose("stand")).translation().z(), 0.01);
  }
}
