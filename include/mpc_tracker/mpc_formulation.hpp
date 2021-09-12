
#ifndef MPC_FORMULATION_H
#define MPC_FORMULATION_H

#include <array>
#include <cassert>
#include <cmath>
#include <functional>
#include <vector>
#include "mpc_tracker/state_space_order.hpp"

namespace cgmres {
// This class stores parameters of NMPC and equations of NMPC.
class NMPCModel {
 private:
  static constexpr int dim_state_ = MPC_STATE_SPACE::DIM;
  static constexpr int dim_control_input_ = MPC_INPUT::DIM;
  static constexpr int dim_constraints_ = 0;

  std::array<double, dim_state_> q_ = {0.0, 0.1, 0.1, 0.1};
  std::array<double, dim_state_> q_terminal_ = {0.0, 0.1, 0.1, 0.1};
  std::array<double, dim_state_> x_ref_ = {0.0, 0.0, 0.0, 0.0};
  std::array<double, dim_control_input_> r_ = {0.01, 0.01};

  double barrier_coefficient_ = 0.1;
  double a_max_ = 0.2;
  double a_min_ = -0.2;
  // const double gamma_ = 0.0;

 public:
  // Computes the state equation f(t, x, u).
  // t : time parameter
  // x : state vector
  // u : control input vector
  // f : the value of f(t, x, u)
  void stateFunc(const double t, const std::vector<double> &x, const double *u,
                 std::function<double(double)> &traj_curvature, std::function<double(double)> &traj_speed,
                 std::function<double(double)> &drivable_width, std::vector<double> &dx) const;

  // Computes the partial derivative of terminal cost with respect to state,
  // i.e., dphi/dx(t, x).
  // t    : time parameter
  // x    : state vector
  // phix : the value of dphi/dx(t, x)
  void phixFunc(const double t, const std::vector<double> &x, std::function<double(double)> &traj_curvature,
                std::function<double(double)> &traj_speed, std::function<double(double)> &drivable_width,
                std::vector<double> &phix) const;

  // Computes the partial derivative of the Hamiltonian with respect to state,
  // i.e., dH/dx(t, x, u, lmd).
  // t   : time parameter
  // x   : state vector
  // u   : control input vector
  // lmd : the Lagrange multiplier for the state equation
  // hx  : the value of dH/dx(t, x, u, lmd)
  void hxFunc(const double t, const std::vector<double> &x, const double *u, const std::vector<double> &lmd,
              std::function<double(double)> &traj_curvature, std::function<double(double)> &traj_speed,
              std::function<double(double)> &drivable_width, std::vector<double> &hx) const;

  // Computes the partial derivative of the Hamiltonian with respect to control
  // input and the constraints, dH/du(t, x, u, lmd).
  // t   : time parameter
  // x   : state vector
  // u   : control input vector
  // lmd : the Lagrange multiplier for the state equation
  // hu  : the value of dH/du(t, x, u, lmd)
  void huFunc(const double t, const std::vector<double> &x, const double *u, const std::vector<double> &lmd,
              std::function<double(double)> &traj_curvature, std::function<double(double)> &traj_speed,
              std::function<double(double)> &drivable_width, double *hu) const;

  // Returns the dimension of the state.
  int dim_state() const;

  // Returns the dimension of the contorl input.
  int dim_control_input() const;

  // Returns the dimension of the constraints.
  int dim_constraints() const;

  // Set parameters
  void set_parameters(const std::array<double, MPC_STATE_SPACE::DIM> &q,
                      const std::array<double, MPC_STATE_SPACE::DIM> &q_terminal,
                      const std::array<double, MPC_INPUT::DIM> &r, const double barrier_coefficient, const double a_max,
                      const double a_min);
};

}  // namespace cgmres

#endif  // MPC_FORMULATION_H
