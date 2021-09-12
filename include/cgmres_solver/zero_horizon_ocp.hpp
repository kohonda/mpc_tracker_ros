// Privides the optimal control problem (OCP) with horizon whose length is zero.

#ifndef ZERO_HORIZON_OCP_H
#define ZERO_HORIZON_OCP_H

#include "cgmres_solver/linear_algebra.hpp"
#include "cgmres_solver/optimal_control_problem.hpp"

namespace cgmres {
// Privides the optimal control problem (OCP) with horizon whose length is zero.
class ZeroHorizonOCP final : public OptimalControlProblem {
 public:
  // Allocate a vector.
  ZeroHorizonOCP();

  // Free a vector.
  ~ZeroHorizonOCP();

  // Computes the optimaliy residual under time, state_vec, and solution_vec
  // that represents the control input and Lgrange multiplier with respect to
  // equality constraints. The result is set in optimality_residual.
  void computeOptimalityResidual(const double time, const std::vector<double>& state_vec, const double* solution_vec,
                                 std::function<double(double)>& traj_curvature,
                                 std::function<double(double)>& traj_speed,
                                 std::function<double(double)>& drivable_width, double* optimality_residual);

  // Computes the partial derivative of the terminal cost with respect to
  // the state.
  void computeTerminalCostDerivative(const double time, const std::vector<double>& state_vec,
                                     std::function<double(double)>& traj_curvature,
                                     std::function<double(double)>& traj_speed,
                                     std::function<double(double)>& drivable_width,
                                     std::vector<double>& terminal_cost_derivative_vec);

  // Return the dimension of the solution,
  // i.e., dim_control_input+dim_constraints.
  int dim_solution() const override;

 private:
  int dim_solution_;
  std::vector<double> lambda_vec_;
};

}  // namespace cgmres

#endif  // ZERO_HORIZON_OCP_H