import sys
sys.dont_write_bytecode = True

import subprocess
import platform

import sympy
from sympy import sin, cos, tan, exp, log, sinh, cosh, tanh, atan, diff, sqrt, Piecewise, Max

from autogenu import symbolic_functions as symfunc


class AutoGenU(object):
    """ Automatic C++ code generator for the C/GMRES methods. 

        Args: 
            model_name: The name of the NMPC model. The directory having this 
                name is made and C++ source files are generated in the 
                directory.
            dimx: The dimension of the state of the NMPC model. 
            dimu: The dimension of the control input of the NMPC model. 
    """
    def __init__(self, model_name, dim_ego_x, dim_other_x, dimu):
        assert isinstance(model_name, str), 'The frst argument must be strings!'
        assert dim_ego_x > 0, 'The second argument must be positive integer!'
        assert dimu > 0, 'The third argument must be positive integer!'
        self.__model_name = model_name
        self.__dim_ego_x = dim_ego_x
        self.__dim_others_x = dim_other_x        
        self.__dimu = dimu        
        self.__scalar_vars = []
        self.__array_vars = []
        self.__is_function_set = False
 

    def define_t(self):
        """ Returns symbolic scalar variable 't'.
        """
        return sympy.Symbol('t')


    def define_scalar_var(self, scalar_var_name):
        """ Returns symbolic variable whose name is scalar_var_name. The name of 
            the variable is memorized.

            Args:
                scalar_var_name: Name of the scalar variable.
        """
        assert isinstance(scalar_var_name, str), 'The input must be strings!'
        scalar_var = sympy.Symbol(scalar_var_name)
        self.__scalar_vars.append([scalar_var, scalar_var_name, 0])
        return scalar_var

    def define_scalar_vars(self, *scalar_var_name_list):
        """ Returns symbolic variables whose names are given by 
            scalar_var_name_list. The names of the variables are memorized.

            Args:
                scalar_var_name_list: Names of the scalar variables.
        """
        scalar_vars = []
        for scalar_var_name in scalar_var_name_list:
            assert isinstance(scalar_var_name, str), 'The input must be list of strings!'
            scalar_var = sympy.Symbol(scalar_var_name)
            self.__scalar_vars.append([scalar_var, scalar_var_name, 0])
            scalar_vars.append(scalar_var)
        return scalar_vars

    def define_array_var(self, array_var_name, dim):
        """ Returns symbolic vector variable whose names is array_var_name and 
            whose dimension is dim. The names of the variable is memorized.

            Args:
                array_var_name: Name of the array variable.
                dim: Dimension of the array variable.
        """
        assert isinstance(array_var_name, str), 'The first argument must be strings!'
        assert dim > 0, 'The second argument must be positive integer!'
        array_var = sympy.symbols(array_var_name+'[0:%d]' %(dim))
        self.__array_vars.append([array_var, array_var_name, []])
        return array_var


    def set_scalar_var(self, scalar_var_name, scalar_value):
        """ Set the value of the scalar variable you defied. 

            Args:
                scalar_var_name: Name of the scalar variable.
                scalar_value: Value of the scalar variable.
        """
        assert isinstance(scalar_var_name, str), 'The first argument must be strings!'
        for defined_scalar_var in self.__scalar_vars:
            if scalar_var_name[0] == defined_scalar_var[1]:
                defined_scalar_var[2] = scalar_value

    def set_scalar_vars(self, *scalar_var_name_and_value_list):
        """ Set the values of the scalar variables you defied. 

            Args:
                scalar_var_name_and_value_lis: A list composed of the name of 
                the scalar variable and value of the scalar variable.
        """
        for var_name_and_value in scalar_var_name_and_value_list:
            for defined_scalar_var in self.__scalar_vars:
                if var_name_and_value[0] == defined_scalar_var[1]:
                    defined_scalar_var[2] = var_name_and_value[1]
    
    def set_array_var(self, var_name, values):
        """ Set the value of the array variable you defied. 

            Args:
                var_name: Name of the arrray variable.
                values: Values of the array variable. The size must be the 
                    dimension of the array variable.
        """
        assert isinstance(var_name, str), 'The first argument must be strings!'
        for defined_array_var in self.__array_vars:
            if var_name == defined_array_var[1]:
                if len(defined_array_var[0]) == len(values):
                    defined_array_var[2] = values

    def set_functions(self, ego_state, ego_input, others_state, f_ego, ineq_constraints, L_ego, phi):
        """ Sets functions that defines the optimal control problem.
        # TODO　説明分の更新必要
            Args: 
                f: The state equation. The dimension must be dimx.
                C: The equality constraints. If there are no equality 
                    constraints, set the empty list.
                h: The inequality constraints considered by semi-smooth 
                    Fischer-Burumeister method. If there are no such inequality 
                    constraints, set the empty list.
                L: The stage cost.
                phi: The terminal cost.
        """
        assert ego_state.DIM_STATE == len(f_ego)


        self.__state_func = list(f_ego.values())   
        self.__ego_state_name_list = ego_state.state_name_list
        self.__state_name_list = self.__ego_state_name_list 
        self.__input_name_list = ego_input.input_name_list


        # 1. x,u, lmd等のsymbolを用意
        lmd_ego = ego_state.generate_lmd()

        # 2. 緩和付きlogバリア関数を定義
        log_barrier_with_relaxation = []
        for g in ineq_constraints:
            # 係数も自動化したいのでとりあえず以下のようにする
            # TODO : ペナルティとバリア関数の係数を調整する
            # mu_g = 1/g# ペナルティ関数の係数
            # rho_g = exp(g)# バリア関数の係数
            # mu_g = 0# ペナルティ関数の係数
            rho_g = sympy.Symbol('rho_g_')# バリア関数の係数 

            ## relaxed log Method : ２つの関数の接合部が滑らかなので不安定になりにくい
            # https://www.researchgate.net/publication/273471963_Relaxed_Logarithmic_Barrier_Function_Based_Model_Predictive_Control_of_Linear_Systems
            delta = 1.0
            k=2# 偶数
            barrier_func = - rho_g * log(-g)
            beta = rho_g * (k-1)/k * (((-g-k*delta)/((k-1)*delta))**2 - 1)-log(delta)
            relaxed_log_barrier = Piecewise((barrier_func, g < -delta), (beta, True))
            
            ## ペナルティ関数法 : 不安定になることがある
            # g<=0 <=> if(g<0){-rho_g * ln(-g)}, else{mu_g * max(0,g)^2}
            # penalty_func = mu_g * Max(0,g)**2
            # relaxed_log_barrier = Piecewise((barrier_func, g<0), (penalty_func, g>=0))
            
            log_barrier_with_relaxation.append(relaxed_log_barrier)

        # 3. ハミルトニアンを定義
        hamiltonian = L_ego +  sum(lmd_ego[key]*f_ego[key] for key in ego_state.state_dict) 
        hamiltonian += sum(constraint for constraint in log_barrier_with_relaxation)

        
        # 4. ハミルトニアンを微分

        # Hx_ego
        self.__hx_ego = []
        for x_ego in ego_state.state_dict.values():
            self.__hx_ego.append(sympy.diff(hamiltonian, x_ego))

        # Hu
        self.__hu=[]
        for input in ego_input.input_dict.values():
            self.__hu.append(sympy.diff(hamiltonian, input))

        # phix, x = [x_ego, x_others]
        self.__phix = []
        for x_ego in ego_state.state_dict.values():
            self.__phix.append(sympy.diff(phi, x_ego))

        self.__is_function_set = True
    


    def generate_source_files(self, use_simplification=False, use_cse=False):
        """ Generates the C++ source file in which the equations to solve the 
            optimal control problem are described. Before call this method, 
            set_functions() must be called.

            Args: 
                use_simplification: The flag for simplification. If True, the 
                    Symbolic functions are simplified. Default is False.
                use_cse: The flag for common subexpression elimination. If True, 
                    common subexpressions are eliminated. Default is False.
        """
        assert self.__is_function_set, "Symbolic functions are not set!. Before call this method, call set_functions()"
        
        self.__make_model_dir()
        if use_simplification:
            symfunc.simplify(self.__state_func)
            symfunc.simplify(self.__hx_ego)
            symfunc.simplify(self.__hu)
            symfunc.simplify(self.__phix)

        f_model_h = open('generated_c_source/'+str(self.__model_name)+'/' + str(self.__model_name) +'.hpp', 'w')
        f_model_h.writelines([
""" 
# pragma once

#include <cmath>
#include <array>
#include <vector>
#include "mpc_tracker/state_space_order.hpp"

namespace cgmres {

// This class stores parameters of NMPC and equations of NMPC.

class NMPCModel {
private:

"""
        ])

       
        f_model_h.writelines([
"""

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

""" 

        ])
        
        f_model_h.write('\n')
        f_model_h.writelines('private:\n')

        f_model_h.write(
            '  static constexpr int dim_ego_state_ = EGO_STATE_SPACE::DIM;\n'
        )
        f_model_h.write(
            '  static constexpr int dim_control_input_ =  EGO_INPUT::DIM;\n'
        )
        f_model_h.write(
            str(self.__model_name) + '();\n'
        )
        f_model_h.writelines([
            '  static constexpr double '+scalar_var[1]+' = '
            +str(scalar_var[2])+';\n' for scalar_var in self.__scalar_vars
        ])
        
        f_model_h.write('\n')

        for array_var in self.__array_vars:
            f_model_h.write(
                '  double '+array_var[1]+'['+str(len(array_var[0]))+']'+' = {'
            )
            for i in range(len(array_var[0])-1):
                f_model_h.write(str(array_var[2][i])+', ')
            f_model_h.write(str(array_var[2][len(array_var[0])-1])+'};\n')
        

        f_model_h.writelines([
""" 
    std::array<double, dim_ego_state_> q_;
    std::array<double, dim_ego_state_> q_terminal_;
    std::array<double, dim_ego_state_> x_ref_;
    std::array<double, dim_control_input_> r_;
"""
        ])
        
        f_model_h.writelines([
"""

};

} // namespace cgmres

""" 


        ])
        
        f_model_h.close()
        f_model_c =  open('generated_c_source/'+str(self.__model_name)+'/' + str(self.__model_name) + '.cpp', 'w')
        f_model_c.write('#include "'+ 'mpc_tracker/mpc_formulation.hpp"\n')
        f_model_c.writelines('\n') 
        f_model_c.writelines('namespace cgmres {\n')
        f_model_c.writelines('void '+ 'NMPCModel::state_func(const double t, const std::vector<double>& x, const double* u, std::function<double(double)>& traj_curvature, std::function<double(double)>& traj_speed, std::function<double(double)>& drivable_width, std::vector<double>& dx) const{')
        # f_model_h.writelines('const double curvature = traj_curvature(x[' + str(self.__state_name_list['x_f'])+ ']);')
        self.__write_function(f_model_c, self.__state_func, self.__state_name_list,'dx', use_cse)
        f_model_c.writelines([
""" 
}
"""
        ])
        f_model_c.writelines('void '+ 'NMPCModel::phixFunc(const double& t, const std::vector<double>& x, const double& curvature, const double& ref_speed, const double& drivable_width) const{')
        self.__write_function(f_model_c, self.__phix, self.__state_name_list ,'phix', use_cse)
        f_model_c.writelines([
""" 
}
"""
        ])
        f_model_c.writelines('void '+ 'NMPCModel::hxFunc(const double& t, const std::vector<double>& x, const double* u, const std::vector<double>& lmd, const double& curvature, const double& ref_speed, const double& drivable_width) const')
        f_model_c.writelines([
"""
{
"""
        ])
        self.__write_function(f_model_c, self.__hx_ego, self.__ego_state_name_list, 'hx', use_cse)
        f_model_c.writelines([
""" 
}
"""
        ])
        
        f_model_c.writelines('void '+ 'NMPCModel::huFunc(const double& t, const std::vector<double>& x, const double* u, const std::vector<double>& lmd, const double& curvature, const double& ref_speed, const double& drivable_width) const')
        f_model_c.writelines([
"""
{
"""
        ])
        self.__write_function(f_model_c, self.__hu, self.__input_name_list ,'hu', use_cse)
        f_model_c.writelines([
"""
}


} // namespace cgmres

""" 
        ])
        f_model_c.close() 

    def __write_function(
            self, writable_file, function_list, name_list ,return_value_name, use_cse
        ):
        """ Write input symbolic function onto writable_file. The function's 
            return value name must be set. use_cse is optional.

            Args: 
                writable_file: A writable file, i.e., a file streaming that is 
                    already opened as writing mode.
                function: A symbolic function wrote onto the writable_file.
                return_value_name: The name of the return value.
                use_cse: If true, common subexpression elimination is used. If 
                    False, it is not used.
        """
        assert len(function_list) == len(name_list), 'The function and the list of state names must be connected.'
        if use_cse:
            func_cse = sympy.cse(function_list)
            for i in range(len(func_cse[0])):
                cse_exp, cse_rhs = func_cse[0][i]
                writable_file.write(
                    '  double '+sympy.ccode(cse_exp)
                    +' = '+sympy.ccode(cse_rhs)+';\n'
                )
            for i in range(len(func_cse[1])):
                writable_file.write(
                    '  '+return_value_name+'[%s] = '%(name_list[i])
                    +sympy.ccode(func_cse[1][i])+';\n'
                )
        else:
            writable_file.writelines(
                ['  '+return_value_name+'[%s] = '%(name_list[i])
                +sympy.ccode(function_list[i])+';\n' for i in range(len(function_list))]
            )
    
   

    def __make_model_dir(self):
        """ Makes a directory where the C source files of OCP models are 
            generated.
        """
        if platform.system() == 'Windows':
            subprocess.run(
                ['mkdir', 'models'], 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                shell=True
            )
            subprocess.run(
                ['mkdir', self.__model_name], 
                cwd='models', 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                shell=True
            )
        else:
            subprocess.run(
                ['mkdir', 'generated_c_source'], 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE
            )
            subprocess.run(
                ['mkdir', self.__model_name], 
                cwd='generated_c_source',
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE
            )

