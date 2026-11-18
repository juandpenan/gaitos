//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_dynamics/OsqpSolver.hpp"

#include <osqp.h>

#include <algorithm>

namespace gaitos_dynamics
{

namespace
{
/// Compressed sparse column storage owned by us (OSQP only references it)
struct Csc
{
  std::vector<OSQPFloat> x;
  std::vector<OSQPInt> i;
  std::vector<OSQPInt> p;
  OSQPCscMatrix m{};

  /// Build from a dense matrix; upper: keep only the upper triangle (P of OSQP)
  void build(const Eigen::MatrixXd & d, bool upper)
  {
    x.clear();
    i.clear();
    p.assign(static_cast<std::size_t>(d.cols() + 1), 0);
    for (Eigen::Index c = 0; c < d.cols(); ++c) {
      const Eigen::Index last = upper ? std::min(c, d.rows() - 1) : d.rows() - 1;
      for (Eigen::Index r = 0; r <= last; ++r) {
        // P keeps every upper entry so that its pattern does not change between calls
        if (upper || d(r, c) != 0.0) {
          x.push_back(d(r, c));
          i.push_back(static_cast<OSQPInt>(r));
        }
      }
      p[static_cast<std::size_t>(c + 1)] = static_cast<OSQPInt>(x.size());
    }
    OSQPCscMatrix_set_data(&m, static_cast<OSQPInt>(d.rows()), static_cast<OSQPInt>(d.cols()),
      static_cast<OSQPInt>(x.size()), x.data(), i.data(), p.data());
  }

  bool same_pattern(const Csc & o) const {return i == o.i && p == o.p;}
};
}  // namespace

struct OsqpSolver::Impl
{
  OSQPSolver * solver{nullptr};
  Csc P;
  Csc A;
  Eigen::Index n{-1};
  Eigen::Index m{-1};

  ~Impl() {clear();}
  void clear()
  {
    if (solver) {
      osqp_cleanup(solver);
      solver = nullptr;
    }
  }
};

OsqpSolver::OsqpSolver()
: impl_(std::make_unique<Impl>()) {}

OsqpSolver::~OsqpSolver() = default;

void OsqpSolver::reset()
{
  impl_->clear();
}

OsqpSolver::Result OsqpSolver::solve(
  const Eigen::MatrixXd & P, const Eigen::VectorXd & q, const Eigen::MatrixXd & A,
  const Eigen::VectorXd & l, const Eigen::VectorXd & u, Eigen::VectorXd & x)
{
  auto & s = *impl_;
  Csc p_new, a_new;
  p_new.build(P, true);
  a_new.build(A, false);
  // OSQP uses +-OSQP_INFTY for missing bounds
  const Eigen::VectorXd lo = l.cwiseMax(-OSQP_INFTY);
  const Eigen::VectorXd hi = u.cwiseMin(OSQP_INFTY);

  Result r;
  const bool reuse = s.solver && s.n == P.rows() && s.m == A.rows() &&
    p_new.same_pattern(s.P) && a_new.same_pattern(s.A);
  if (reuse) {
    s.P.x = p_new.x;
    s.A.x = a_new.x;
    osqp_update_data_mat(s.solver, s.P.x.data(), OSQP_NULL, static_cast<OSQPInt>(s.P.x.size()),
      s.A.x.data(), OSQP_NULL, static_cast<OSQPInt>(s.A.x.size()));
    osqp_update_data_vec(s.solver, q.data(), lo.data(), hi.data());
  } else {
    s.clear();
    s.P = std::move(p_new);
    s.A = std::move(a_new);
    // The matrices moved: point them to their (moved) storage again
    OSQPCscMatrix_set_data(&s.P.m, static_cast<OSQPInt>(P.rows()), static_cast<OSQPInt>(P.cols()),
      static_cast<OSQPInt>(s.P.x.size()), s.P.x.data(), s.P.i.data(), s.P.p.data());
    OSQPCscMatrix_set_data(&s.A.m, static_cast<OSQPInt>(A.rows()), static_cast<OSQPInt>(A.cols()),
      static_cast<OSQPInt>(s.A.x.size()), s.A.x.data(), s.A.i.data(), s.A.p.data());
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = settings_.verbose ? 1 : 0;
    settings.warm_starting = 1;
    settings.polishing = settings_.polishing ? 1 : 0;
    settings.max_iter = settings_.max_iterations;
    settings.eps_abs = settings_.eps_abs;
    settings.eps_rel = settings_.eps_rel;
    if (osqp_setup(&s.solver, &s.P.m, q.data(), &s.A.m, lo.data(), hi.data(),
      static_cast<OSQPInt>(A.rows()), static_cast<OSQPInt>(P.rows()), &settings) != 0)
    {
      s.clear();
      return r;
    }
    s.n = P.rows();
    s.m = A.rows();
    if (x.size() == P.rows()) {
      osqp_warm_start(s.solver, x.data(), OSQP_NULL);
    }
  }
  if (osqp_solve(s.solver) != 0) {
    return r;
  }
  const OSQPInfo * info = s.solver->info;
  r.status = static_cast<int>(info->status_val);
  r.iterations = static_cast<int>(info->iter);
  r.primal_residual = info->prim_res;
  r.dual_residual = info->dual_res;
  r.converged = info->status_val == OSQP_SOLVED || info->status_val == OSQP_SOLVED_INACCURATE ||
    info->status_val == OSQP_MAX_ITER_REACHED;
  if (r.converged) {
    x = Eigen::Map<const Eigen::VectorXd>(s.solver->solution->x, P.rows());
  }
  r.converged = info->status_val == OSQP_SOLVED || info->status_val == OSQP_SOLVED_INACCURATE;
  return r;
}

}  // namespace gaitos_dynamics
