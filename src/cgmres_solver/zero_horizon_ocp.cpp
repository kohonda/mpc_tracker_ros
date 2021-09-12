#include "cgmres_solver/zero_horizon_ocp.hpp"

namespace cgmres {
ZeroHorizonOCP::ZeroHorizonOCP()
    : OptimalControlProblem(),
      dim_solution_(model_.dim_control_input() + model_.dim_constraints()),
      lambda_vec_(dim_state_) {}

ZeroHorizonOCP::~ZeroHorizonOCP() { std::vector<double>().swap(lambda_vec_); }

void ZeroHorizonOCP::computeOptimalityResidual(const double time, const std::vector<double>& state_vec,
                                               const double* solution_vec,
                                               std::function<double(double)>& traj_curvature,
                                               std::function<double(double)>& traj_speed,
                                               std::function<double(double)>& drivable_width,
                                               double* optimality_residual) {
  model_.phixFunc(time, state_vec, traj_curvature, traj_speed, drivable_width, lambda_vec_);
  model_.huFunc(time, state_vec, solution_vec, lambda_vec_, traj_curvature, traj_speed, drivable_width,
                optimality_residual);
}

void ZeroHorizonOCP::computeTerminalCostDerivative(const double time, const std::vector<double>& state_vec,
                                                   std::function<double(double)>& traj_curvature,
                                                   std::function<double(double)>& traj_speed,
                                                   std::function<double(double)>& drivable_width,
                                                   std::vector<double>& terminal_cost_derivative_vec) {
  model_.phixFunc(time, state_vec, traj_curvature, traj_speed, drivable_width, terminal_cost_derivative_vec);
}

int ZeroHorizonOCP::dim_solution() const { return dim_solution_; }

}  // namespace cgmres