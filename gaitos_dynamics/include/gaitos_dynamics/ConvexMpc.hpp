//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_DYNAMICS__CONVEXMPC_HPP_
#define GAITOS_DYNAMICS__CONVEXMPC_HPP_

#include <Eigen/Dense>

#include <vector>

#include "gaitos_dynamics/OsqpSolver.hpp"

namespace gaitos_dynamics
{

using Vector13 = Eigen::Matrix<double, 13, 1>;

/**
 * @class ConvexMpc
 * @brief Convex model predictive control of the ground reaction forces, after Di Carlo et
 * al., "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control",
 * IROS 2018.
 *
 * Single rigid body, state x = [roll, pitch, yaw, p (world), omega (world), v (world), g],
 * inputs the ground reaction force of every foot (world). Dynamics linearized around the
 * current yaw, discretized at `dt`, condensed over the horizon into a QP with friction
 * pyramids and normal force bounds (zero for feet in swing).
 */
class ConvexMpc
{
public:
  struct Parameters
  {
    double mass{15.0};
    Eigen::Matrix3d inertia{Eigen::Matrix3d::Identity() * 0.1};   ///< Body frame, about the CoM
    int horizon{10};
    double dt{0.03};
    double mu{0.5};
    double fz_min{0.0};
    double fz_max{150.0};
    Vector13 weights{(Vector13() << 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1, 0).finished()};
    double force_weight{1e-6};   // per N^2 (the 4e-5 of the paper lifts the body)
    double gravity{9.81};
  };

  void configure(const Parameters & p, int legs);
  const Parameters & parameters() const {return p_;}

  /**
   * @param x0 current state
   * @param x_ref reference, 13 * horizon (state after each step)
   * @param feet foot positions relative to the CoM (world axes), one per leg
   * @param contact contact[k][leg] over the horizon
   * @param[out] forces ground reaction force of each leg at the first step (world)
   * @return the QP result
   */
  OsqpSolver::Result solve(
    const Vector13 & x0, const Eigen::VectorXd & x_ref, const std::vector<Eigen::Vector3d> & feet,
    const std::vector<std::vector<bool>> & contact, std::vector<Eigen::Vector3d> & forces);

  /// Predicted states of the last solution (13 * horizon)
  const Eigen::VectorXd & prediction() const {return prediction_;}

private:
  Parameters p_;
  int legs_{4};
  OsqpSolver qp_;
  Eigen::VectorXd u_;
  Eigen::VectorXd prediction_;
};

}  // namespace gaitos_dynamics

#endif  // GAITOS_DYNAMICS__CONVEXMPC_HPP_
