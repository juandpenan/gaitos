//  Copyright 2026 Intelligent Robotics Lab (URJC)
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <cmath>

#include "gaitos_mujoco/MujocoRenderer.hpp"

using gaitos_mujoco::MujocoRenderer;

TEST(Renderer, LinearDepthInvertsTheReversedDepth)
{
  // Values read from MuJoCo 3.5 (mjDEPTH_ZEROFAR) for a floor at 1 m and 4 m, near 0.02, far 50
  EXPECT_NEAR(MujocoRenderer::linear_depth(0.019608f, 0.02, 50.0), 1.0, 1e-4);
  EXPECT_NEAR(MujocoRenderer::linear_depth(0.004602f, 0.02, 50.0), 4.0, 1e-3);
  EXPECT_NEAR(MujocoRenderer::linear_depth(1.0f, 0.02, 50.0), 0.02, 1e-9);   // near plane
  EXPECT_EQ(MujocoRenderer::linear_depth(0.0f, 0.02, 50.0), 0.0);            // nothing
}

TEST(Renderer, CameraPoseFromTheOpticalFrame)
{
  // Optical frame (z forward, y down) looking along the body x, level: rpy (-pi/2, 0, -pi/2)
  MujocoRenderer::Camera c;
  c.xyz = {0.1, 0.0, 0.2};
  const double r = -M_PI / 2, y = -M_PI / 2;   // Rz(y) Rx(r)
  c.rotation = {std::cos(y), -std::sin(y) * std::cos(r), std::sin(y) * std::sin(r),
    std::sin(y), std::cos(y) * std::cos(r), -std::cos(y) * std::sin(r),
    0.0, std::sin(r), std::cos(r)};
  // Body at (1, 2, 3) turned 90 deg left
  const mjtNum p[3] = {1.0, 2.0, 3.0};
  const mjtNum R[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  float pos[3], forward[3], up[3];
  MujocoRenderer::camera_pose(c, p, R, pos, forward, up);
  EXPECT_NEAR(pos[0], 1.0, 1e-6);
  EXPECT_NEAR(pos[1], 2.1, 1e-6);
  EXPECT_NEAR(pos[2], 3.2, 1e-6);
  EXPECT_NEAR(forward[0], 0.0, 1e-6);   // body x = world y
  EXPECT_NEAR(forward[1], 1.0, 1e-6);
  EXPECT_NEAR(up[2], 1.0, 1e-6);        // image up = world up
}
