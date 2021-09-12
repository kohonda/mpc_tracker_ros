// The single-shooting two-point boundary-value problem of the
// finite-horizon optimal control problem.

#ifndef SINGLE_SHOOTING_OCP_H
#define SINGLE_SHOOTING_OCP_H

#include "cgmres_solver/linear_algebra.hpp"
#include "cgmres_solver/optimal_control_problem.hpp"
#include "cgmres_solver/time_varying_smooth_horizon.hpp"

namespace cgmres {
// Provides the single-shooting two-point boundary-value problem of the
// finite-horizon optimal control problem.
class SingleShootingOCP final : public OptimalControlProblem {
 public:
  // Constructs SingleShootingOCP with setting parameters and allocates
  // vectors and matrices.
  // Arguments:
  //  T_f, alpha: Parameters for the length of the horizon. The length horizon
  //    at time t is given by T_f * (1-exp(-alpha*t)).
  //  N: The number of the discretization of the horizon.
  SingleShootingOCP(const double T_f, const double alpha, const int N);

  // Constructs SingleShootingOCP with setting parameters and allocates
  // vectors and matrices.
  // Arguments:
  //  T_f, alpha: Parameters for the length of the horizon. The length horizon
  //    at time t is given by T_f * (1-exp(-alpha*t)).
  //  N: The number of the discretization of the horizon.
  //  initial_time: Initial time for the length of the horizon.
  SingleShootingOCP(const double T_f, const double alpha, const int N, const double initial_time);

  // Free vectors and matrices.
  ~SingleShootingOCP();

  // Computes the optimaliy residual under time, state_vec, and solution_vec
  // that represents the control input sequence. The result is set in
  // optimality_residual.
  void computeOptimalityResidual(const double time, const std::vector<double>& state_vec, const double* solution_vec,
                                 std::function<double(double)>& traj_curvature,
                                 std::function<double(double)>& traj_speed,
                                 std::function<double(double)>& drivable_width, double* optimality_residual);

  // Predicts the state in the finite future. Under time,state_vec, and
  // solution_vec that represents the control input sequence., and
  // prediction_length The result is set in predicted_state.
  void predictStateFromSolution(const double current_time, const std::vector<double>& current_state,
                                const double* solution_vec, const double prediction_length,
                                std::function<double(double)>& traj_curvature,
                                std::function<double(double)>& traj_speed,
                                std::function<double(double)>& drivable_width, std::vector<double>& predicted_state);

  // Reset the length of the horizon by resetting parameters related to the
  // horizon.
  void resetHorizonLength(const double T_f, const double alpha, const double initial_time);

  // Reset the length of the horizon by resetting parameters related to the
  // horizon.
  void resetHorizonLength(const double initial_time);

  // Returns the dimension of the solution, which is equivalent to
  // N*(dim_control_input+dim_constraints).
  int dim_solution() const override;

  // Returns the grid number of the horizon.
  int N() const;

 private:
  TimeVaryingSmoothHorizon horizon_;
  int dim_solution_, N_;
  // double *dx_vec_, **state_mat_, **lambda_mat_;

  std::vector<double> dx_vec_;
  std::vector<std::vector<double>> state_mat_, lambda_mat_;
};

}  // namespace cgmres

#endif  // SINGLE_SHOOTING_OCP_H