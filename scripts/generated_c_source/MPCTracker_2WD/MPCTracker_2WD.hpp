 
# pragma once

#include <cmath>
#include <array>
#include <vector>
#include "mpc_tracker/state_space_order.hpp"

namespace cgmres {

// This class stores parameters of NMPC and equations of NMPC.

class NMPCModel {
private:



public:

// Computes the state equation f(t, x, u).
  // t : time parameter
  // x : state vector
  // u : control input vector
  // f : the value of f(t, x, u)
  void stateFunc(const double t, const double* x, const double* u, 
                 double* dx) const;

  // Computes the partial derivative of terminal cost with respect to state, 
  // i.e., dphi/dx(t, x).
  // t    : time parameter
  // x    : state vector
  // phix : the value of dphi/dx(t, x)
  void phixFunc(const double t, const double* x, double* phix) const;

  // Computes the partial derivative of the Hamiltonian with respect to state, 
  // i.e., dH/dx(t, x, u, lmd).
  // t   : time parameter
  // x   : state vector
  // u   : control input vector
  // lmd : the Lagrange multiplier for the state equation
  // hx  : the value of dH/dx(t, x, u, lmd)
  void hxFunc(const double t, const double* x, const double* u, 
              const double* lmd, double* hx) const;

  // Computes the partial derivative of the Hamiltonian with respect to control 
  // input and the constraints, dH/du(t, x, u, lmd).
  // t   : time parameter
  // x   : state vector
  // u   : control input vector
  // lmd : the Lagrange multiplier for the state equation
  // hu  : the value of dH/du(t, x, u, lmd)
  void huFunc(const double t, const double* x, const double* u, 
              const double* lmd, double* hu) const;

  // Returns the dimension of the state.
  int dim_state() const;

  // Returns the dimension of the contorl input.
  int dim_control_input() const;

  // Returns the dimension of the constraints.
  int dim_constraints() const;


private:
  static constexpr int dim_ego_state_ = EGO_STATE_SPACE::DIM;
  static constexpr int dim_control_input_ =  EGO_INPUT::DIM;
MPCTracker_2WD();

 
    std::array<double, dim_ego_state_> q_;
    std::array<double, dim_ego_state_> q_terminal_;
    std::array<double, dim_ego_state_> x_ref_;
    std::array<double, dim_control_input_> r_;


};

} // namespace cgmres

