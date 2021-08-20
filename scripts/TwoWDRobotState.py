import sympy

class EgoState:
    
    def __init__(self):
        
        # ============= User setting =====================
        self.DIM_STATE = 3
        self.x_f = sympy.Symbol('x[MPC_STATE_SPACE::X_F]')
        self.y_f = sympy.Symbol('x[MPC_STATE_SPACE::Y_F]')
        self.yaw_f = sympy.Symbol('x[MPC_STATE_SPACE::YAW_F]')
        
        self.state_dict = {'x_f':self.x_f, 'y_f':self.y_f, 'yaw_f':self.yaw_f}
        # ============= User setting =====================
        assert self.DIM_STATE > 0
        assert self.DIM_STATE == len(self.state_dict)
        self.state_name_list=[]
        for state in self.state_dict.values():
            self.state_name_list.append(str(state).replace('x[', '').replace(']',''))

        
    def generate_q(self):
        q = {}
        for key in self.state_dict:
            q[key] = sympy.Symbol(str(self.state_dict[key]).replace('x','q_'))
        return q

    def generate_q_terminal(self):
        q_terminal = {}
        for key in self.state_dict:
            q_terminal[key] = sympy.Symbol(str(self.state_dict[key]).replace('x','q_terminal_'))
        return q_terminal
    
    def generate_lmd(self):
        lmd = {}
        for key in self.state_dict:
            lmd[key] = sympy.Symbol(str(self.state_dict[key]).replace('x','lmd'))
        return lmd

    def generate_state_ref(self):
        x_ref = {}
        for key in self.state_dict:
            x_ref[key] = sympy.Symbol(str(self.state_dict[key]).replace('x','x_ref_'))
        return x_ref
    
    def define_f(self):
        f = {}
        for key in self.state_dict:
            f[key] = None
        return f

class EgoInput:
    
    def __init__(self):
        # ======== User setting ===============
        self.DIM_INPUT = 2
        self.twist_yaw = sympy.Symbol('u[MPC_INPUT::ANGULAR_VEL_YAW]')
        self.twist_x = sympy.Symbol('u[MPC_INPUT::TWIST_X]')

        self.input_dict = {'twist_yaw':self.twist_yaw, 'twist_x':self.twist_x}
        # ======== User setting ===============
        
        assert self.DIM_INPUT > 0
        assert self.DIM_INPUT == len(self.input_dict)
        self.input_name_list=[]
        for input in self.input_dict.values():
            self.input_name_list.append(str(input).replace('u[', '').replace(']',''))
        
    def generate_r(self):
        r = {}
        for key in self.input_dict:
            r[key] = sympy.Symbol(str(self.input_dict[key]).replace('u','r_'))
        return r