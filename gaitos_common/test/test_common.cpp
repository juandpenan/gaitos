//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "gaitos_common/Math.hpp"
#include "gaitos_common/Params.hpp"
#include "gaitos_common/Types.hpp"
#include "gaitos_common/testing.hpp"

using namespace gaitos;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new testutil::RosEnvironment);

TEST(Math, RpyRoundTrip)
{
  const Eigen::Vector3d rpy(0.1, -0.2, 2.5);
  EXPECT_TRUE(quaternion_to_rpy(rpy_to_quaternion(rpy)).isApprox(rpy, 1e-9));
  EXPECT_NEAR(yaw_of(rpy_to_quaternion(rpy)), 2.5, 1e-9);
  EXPECT_NEAR(wrap_angle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(tilt_angle(rpy_to_quaternion({0.3, 0.0, 1.0})), 0.3, 1e-9);
}

TEST(Types, JointCommandTorqueAndButtons)
{
  JointCommand c;
  c.resize(1);
  c.position[0] = 1.0;
  c.kp[0] = 10.0;
  c.kd[0] = 2.0;
  c.effort[0] = 0.5;
  EXPECT_DOUBLE_EQ(c.torque(0, 0.5, 1.0), 10.0 * 0.5 - 2.0 + 0.5);
  EXPECT_EQ(parse_buttons("L2+B"), RemoteButton::L2 | RemoteButton::B);
  EXPECT_EQ(parse_buttons("nope"), 0u);
}

TEST(Params, RosParametersFromFiles)
{
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "gaitos_test_params";
  fs::create_directories(dir);
  std::ofstream(dir / "a.yaml") << "/**:\n  ros__parameters:\n    a: {b: 2.5, i: 3, v: [1, 2, 3], s: 4, name: x}\n"
    "    gains: {legs: {kp: 60.0}, FR_hip_joint: {kp: 80.0}}\n";
  std::ofstream(dir / "b.yaml") << "/**:\n  ros__parameters:\n    a: {b: 7.0}\n";
  const Params p = testutil::make_params({(dir / "a.yaml").string(), (dir / "b.yaml").string()});
  EXPECT_DOUBLE_EQ(p.get<double>("a.b", 0.0), 7.0);       // the later file wins
  EXPECT_DOUBLE_EQ(p.get<double>("a.i", 0.0), 3.0);       // integers as doubles
  EXPECT_EQ(p.get<int>("a.i", 0), 3);
  EXPECT_DOUBLE_EQ(p.get<double>("a.missing", 9.0), 9.0); // default (now declared)
  EXPECT_TRUE(p.has("a.missing"));
  EXPECT_TRUE(p.get_vector3("a.v", Eigen::Vector3d::Zero()).isApprox(Eigen::Vector3d(1, 2, 3)));
  EXPECT_TRUE(p.get_vector3("a.s", Eigen::Vector3d::Zero()).isApprox(Eigen::Vector3d::Constant(4)));
  EXPECT_EQ(p.require<std::string>("a.name"), "x");
  EXPECT_TRUE(p.is_string("a.name"));
  EXPECT_THROW(p.require<double>("nope"), ParamsError);
  EXPECT_THROW(p.get<double>("a.name", 0.0), ParamsError);
  const Params a = p.sub("a");
  EXPECT_DOUBLE_EQ(a.get<double>("b", 0.0), 7.0);
  auto keys = p.sub("gains").keys();
  std::sort(keys.begin(), keys.end());
  EXPECT_EQ(keys, (std::vector<std::string>{"FR_hip_joint", "legs"}));
  EXPECT_TRUE(p.has("gains.legs"));
  EXPECT_FALSE(p.has("gains.arms"));
  fs::remove_all(dir);
}

TEST(Params, PackageUris)
{
  EXPECT_EQ(resolve_uri("/abs/file"), "/abs/file");
  EXPECT_THROW(resolve_uri("package://gaitos_no_such_package/x"), ParamsError);
}
