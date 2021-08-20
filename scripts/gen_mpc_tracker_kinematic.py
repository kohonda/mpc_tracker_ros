import sys
sys.dont_write_bytecode = True

from sympy.core.symbol import Symbol
from sympy.sets.sets import set_function
from sympy import sin, cos, tan, exp, log, sinh, cosh, tanh, atan, diff, sqrt, Piecewise, Max
from autogenu import autogenu
from utils import is_None_dict

from TwoWDRobotState import EgoState, EgoInput


ego_state = EgoState()
ego_input = EgoInput()
MODEL_NAME = 'MPCTracker_2WD'

ag = autogenu.AutoGenU(MODEL_NAME, ego_state.DIM_STATE, 0, ego_input.DIM_INPUT)

# weight parameters
q = ego_state.generate_q()
q_terminal = ego_state.generate_q_terminal()
x_ref = ego_state.generate_state_ref()
r = ego_input.generate_r()


ref_speed = Symbol('ref_speed')
x_ref['vel'] = ref_speed
curvature = Symbol('curvature')
# drivable_width = Symbol('drivable_width')

f_ego = ego_state.define_f()

# Define state function
f_ego['x_f'] = ego_input.twist_x * cos(ego_state.yaw_f)/(1 - curvature * ego_state.y_f)
f_ego['y_f'] = ego_input.twist_x * sin(ego_state.yaw_f)
f_ego['yaw_f'] = ego_input.twist_yaw - (curvature*ego_input.twist_x*cos(ego_state.yaw_f))/(1-curvature*ego_state.y_f)

assert is_None_dict(f_ego), '[Error] Definition of ego state function'


# Define stage cost function
L_ego = sum(q[key]*(ego_state.state_dict[key] - x_ref[key])**2 for key in ego_state.state_dict)/2 + 1/2 * r['twist_yaw'] * ego_input.input_dict['twist_yaw'] ** 2 + 1/2 * r['twist_x'] * (ego_input.input_dict['twist_x']-ref_speed)**2

# Define constraint
# Not support equality constraint, only support inequality constraint
# ineq_constraint <= 0
# D_safe - sqrt(x[N]**2) 
ineq_constraints = [
    # drivable_width/2 - ego_state.y_f,
    # ego_state.y_f - drivable_width/2,
    # ego_input.accel - 2.0,
    # -2.0 - ego_input.accel,
    # ego_input.steer - 0.6,
    # -0.6 - ego_input.steer
]

# Define terminal cost function
phi = sum(q_terminal[key]*(ego_state.state_dict[key] - x_ref[key])**2 for key in ego_state.state_dict)/2

ag.set_functions(ego_state, ego_input, ego_state, f_ego, ineq_constraints, L_ego,  phi)

use_simplification = False
use_cse = False

ag.generate_source_files(use_simplification, use_cse)
