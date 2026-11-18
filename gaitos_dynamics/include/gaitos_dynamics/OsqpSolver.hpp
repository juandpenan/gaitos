//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_DYNAMICS__OSQPSOLVER_HPP_
#define GAITOS_DYNAMICS__OSQPSOLVER_HPP_

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace gaitos_dynamics
{

/**
 * @class OsqpSolver
 * @brief Eigen front end of OSQP (https://osqp.org, version 1.x) for
 *
 *   min 0.5 x' P x + q' x   s.t.   l <= A x <= u
 *
 * Dense matrices are converted to compressed columns (upper triangle of P). The solver is
 * set up on the first call and, while the sizes and sparsity patterns stay the same, later
 * calls only update the data and warm start from the previous solution, which suits
 * receding horizon problems such as the MPC.
 */
class OsqpSolver
{
public:
  struct Settings
  {
    int max_iterations{400};
    double eps_abs{1e-4};
    double eps_rel{1e-4};
    bool polishing{false};
    bool verbose{false};
  };

  struct Result
  {
    bool converged{false};      ///< solved (or solved inaccurate)
    int status{0};              ///< OSQP status value
    int iterations{0};
    double primal_residual{0.0};
    double dual_residual{0.0};
  };

  OsqpSolver();
  ~OsqpSolver();
  OsqpSolver(const OsqpSolver &) = delete;
  OsqpSolver & operator=(const OsqpSolver &) = delete;

  Settings & settings() {return settings_;}

  /**
   * @param[out] x solution (unchanged if OSQP fails)
   */
  Result solve(
    const Eigen::MatrixXd & P, const Eigen::VectorXd & q, const Eigen::MatrixXd & A,
    const Eigen::VectorXd & l, const Eigen::VectorXd & u, Eigen::VectorXd & x);

  /// Release the solver (the next solve sets it up again)
  void reset();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Settings settings_;
};

}  // namespace gaitos_dynamics

#endif  // GAITOS_DYNAMICS__OSQPSOLVER_HPP_
