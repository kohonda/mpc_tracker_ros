
#ifndef NMPC_MODEL_H
#define NMPC_MODEL_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <functional>
#include <cassert>
#include "mpc_tracker/state_space_order.hpp"

namespace cgmres
{
// This class stores parameters of NMPC and equations of NMPC.
class NMPCModel
{
    private:
    static constexpr int dim_state_         = 4;
    static constexpr int dim_control_input_ = 2;
    static constexpr int dim_constraints_   = 0;

    static constexpr double lf_ = 1.11;
    static constexpr double lr_ = 1.66;

    double q[4]          = { 1, 1, 0, 1 };
    double q_terminal[4] = { 10, 10, 0, 10 };
    double x_ref[4]      = { 0, 0, 0, 0 };
    double r[2]          = { 1, 1 };

    public:
    // Computes the state equation f(t, x, u).
    // t : time parameter
    // x : state vector
    // u : control input vector
    // f : the value of f(t, x, u)
    void stateFunc(const double t, const std::vector<double>& x, const double* u, std::function<double(double)>& traj_curvature, std::function<double(double)>& traj_speed,
                   std::function<double(double)>& drivable_width, std::vector<double>& dx) const;

    // Computes the partial derivative of terminal cost with respect to state,
    // i.e., dphi/dx(t, x).
    // t    : time parameter
    // x    : state vector
    // phix : the value of dphi/dx(t, x)
    void phixFunc(const double t, const std::vector<double>& x, std::function<double(double)>& traj_curvature, std::function<double(double)>& traj_speed, std::function<double(double)>& drivable_width,
                  std::vector<double>& phix) const;

    // Computes the partial derivative of the Hamiltonian with respect to state,
    // i.e., dH/dx(t, x, u, lmd).
    // t   : time parameter
    // x   : state vector
    // u   : control input vector
    // lmd : the Lagrange multiplier for the state equation
    // hx  : the value of dH/dx(t, x, u, lmd)
    void hxFunc(const double t, const std::vector<double>& x, const double* u, const std::vector<double>& lmd, std::function<double(double)>& traj_curvature, std::function<double(double)>& traj_speed,
                std::function<double(double)>& drivable_width, std::vector<double>& hx) const;

    // Computes the partial derivative of the Hamiltonian with respect to control
    // input and the constraints, dH/du(t, x, u, lmd).
    // t   : time parameter
    // x   : state vector
    // u   : control input vector
    // lmd : the Lagrange multiplier for the state equation
    // hu  : the value of dH/du(t, x, u, lmd)
    void huFunc(const double t, const std::vector<double>& x, const double* u, const std::vector<double>& lmd, std::function<double(double)>& traj_curvature, std::function<double(double)>& traj_speed,
                std::function<double(double)>& drivable_width, double* hu) const;

    // Returns the dimension of the state.
    int dim_state() const;

    // Returns the dimension of the contorl input.
    int dim_control_input() const;

    // Returns the dimension of the constraints.
    int dim_constraints() const;
};

}  // namespace cgmres

#endif  // NMPC_MODEL_H
