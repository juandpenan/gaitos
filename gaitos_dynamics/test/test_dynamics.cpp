//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <cmath>

#include "gaitos_common/Math.hpp"
#include "gaitos_dynamics/LipmPreview.hpp"
#include "gaitos_model/RobotModel.hpp"
#include "gaitos_common/testing.hpp"
#ifdef GAITOS_WITH_OSQP
#include "gaitos_dynamics/ConvexMpc.hpp"
#include "gaitos_dynamics/OsqpSolver.hpp"
#endif

using namespace gaitos_dynamics;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new gaitos::testutil::RosEnvironment);

#ifdef GAITOS_WITH_OSQP
TEST(OsqpSolver, BoxAndInequalityConstraints)
{
  // min (x0 - 1)^2 + (x1 - 2)^2  s.t. x0 + x1 <= 2, x >= 0  ->  x = (0.5, 1.5)
  Eigen::MatrixXd p = 2.0 * Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd q(2);
  q << -2.0, -4.0;
  Eigen::MatrixXd a(3, 2);
  a << 1, 1, 1, 0, 0, 1;
  Eigen::VectorXd l(3), u(3);
  l << -1e20, 0, 0;
  u << 2, 1e20, 1e20;
  OsqpSolver qp;
  Eigen::VectorXd x;
  const auto r = qp.solve(p, q, a, l, u, x);
  EXPECT_TRUE(r.converged);
  EXPECT_NEAR(x(0), 0.5, 1e-3);
  EXPECT_NEAR(x(1), 1.5, 1e-3);
}

TEST(OsqpSolver, EqualityConstraint)
{
  // min x0^2 + x1^2  s.t. x0 + x1 = 1  ->  (0.5, 0.5)
  Eigen::MatrixXd p = 2.0 * Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd a(1, 2);
  a << 1, 1;
  Eigen::VectorXd b(1);
  b << 1.0;
  OsqpSolver qp;
  Eigen::VectorXd x;
  EXPECT_TRUE(qp.solve(p, q, a, b, b, x).converged);
  EXPECT_NEAR(x(0), 0.5, 1e-3);
  EXPECT_NEAR(x(1), 0.5, 1e-3);
}

#endif  // GAITOS_WITH_OSQP

TEST(LipmPreview, TracksZmpSteps)
{
  LipmPreview lipm;
  lipm.configure(0.005, 0.8, 1.6);
  ASSERT_EQ(lipm.preview_steps(), 320);
  // Kajita's gains decay with the preview time
  EXPECT_GT(std::abs(lipm.preview_gains()[1]), std::abs(lipm.preview_gains()[200]));
  lipm.reset(Eigen::Vector2d::Zero());
  // ZMP reference: 0 until t = 1 s, then alternating +-0.1 m in y every 0.5 s, x advancing
  auto ref = [](double t) {
      if (t < 1.0) {
        return Eigen::Vector2d(0.0, 0.0);
      }
      const int step = static_cast<int>((t - 1.0) / 0.5);
      return Eigen::Vector2d(0.2 * step, step % 2 == 0 ? 0.1 : -0.1);
    };
  double sum_err = 0.0;
  int samples = 0;
  double max_com_y = 0.0;
  for (int k = 0; k < 1200; ++k) {
    const double t = k * 0.005;
    std::vector<Eigen::Vector2d> buf(static_cast<std::size_t>(lipm.preview_steps() + 1));
    for (std::size_t j = 0; j < buf.size(); ++j) {
      buf[j] = ref(t + static_cast<double>(j) * 0.005);
    }
    lipm.step(buf);
    if (t > 1.5) {
      sum_err += (lipm.zmp() - ref(t + 0.005)).norm();
      ++samples;
      max_com_y = std::max(max_com_y, std::abs(lipm.com().y()));
    }
  }
  EXPECT_LT(sum_err / samples, 0.02);   // the ZMP follows its (discontinuous) reference
  EXPECT_LT(max_com_y, 0.1);       // the CoM sways less than the feet
  EXPECT_GT(max_com_y, 0.01);
  EXPECT_GT(lipm.com().x(), 1.4);  // and advances with them
}

#ifdef GAITOS_WITH_OSQP
TEST(ConvexMpc, StandingForcesCarryTheWeight)
{
  ConvexMpc::Parameters p;
  p.mass = 15.0;
  p.inertia = Eigen::Vector3d(0.1, 0.25, 0.3).asDiagonal();
  ConvexMpc mpc;
  mpc.configure(p, 4);
  Vector13 x0;
  x0 << 0, 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0, -9.81;
  Eigen::VectorXd ref(13 * p.horizon);
  for (int k = 0; k < p.horizon; ++k) {
    ref.segment<13>(13 * k) = x0;
  }
  const std::vector<Eigen::Vector3d> feet{{0.19, -0.14, -0.3}, {0.19, 0.14, -0.3},
    {-0.19, -0.14, -0.3}, {-0.19, 0.14, -0.3}};
  std::vector<std::vector<bool>> contact(static_cast<std::size_t>(p.horizon), std::vector<bool>(4, true));
  std::vector<Eigen::Vector3d> f;
  for (int i = 0; i < 3; ++i) {   // warm started iterations
    mpc.solve(x0, ref, feet, contact, f);
  }
  Eigen::Vector3d total = Eigen::Vector3d::Zero();
  for (const auto & fi : f) {
    total += fi;
    EXPECT_NEAR(fi.z(), p.mass * 9.81 / 4.0, 3.0);
    EXPECT_LE(std::abs(fi.x()), p.mu * fi.z() + 1e-3);
  }
  EXPECT_NEAR(total.z(), p.mass * 9.81, 3.0);
  EXPECT_NEAR(total.x(), 0.0, 1.0);

  // Two feet in the air: the diagonal pair carries everything, nothing on the others
  for (auto & c : contact) {
    c = {true, false, false, true};
  }
  for (int i = 0; i < 3; ++i) {
    mpc.solve(x0, ref, feet, contact, f);
  }
  EXPECT_NEAR(f[1].norm(), 0.0, 1e-2);
  EXPECT_NEAR(f[2].norm(), 0.0, 1e-2);
  EXPECT_NEAR(f[0].z() + f[3].z(), p.mass * 9.81, 4.0);
}

#endif  // GAITOS_WITH_OSQP

TEST(Model, RigidBodyOfGo2)
{
  const std::string robots = std::string(GAITOS_SOURCE_DIR) + "/gaitos_description/robots/";
  const auto m = gaitos::RobotModel::from_params(gaitos::testutil::make_params({}, {{"robot.urdf", robots + "go2_description/urdf/go2.urdf"}, {"robot.srdf", robots + "go2_description/srdf/go2.srdf"}}));
  const auto rb = m->tree().rigid_body(m->pose("stand"), "base");
  EXPECT_NEAR(rb.mass, 16.1, 0.5);
  EXPECT_LT(std::abs(rb.com.y()), 0.01);
  EXPECT_GT(rb.inertia(2, 2), rb.inertia(0, 0));   // longer than wide
  EXPECT_GT(rb.inertia.eigenvalues().real().minCoeff(), 0.0);
}
