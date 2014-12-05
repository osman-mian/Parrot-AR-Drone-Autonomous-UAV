import numpy as np
class State:
    def __init__(self):
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))

###########################

class UserCode:
    def __init__(self):
        Kp_xy = 0.0001
        Kp_z = 0.0001
        Kd_xy = 0.00005
        Kd_z = 0.00005
        
        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T

###########################
    
    def compute_control_command(self, state, state_desired):
        u = self.Kp * (state_desired.position - state.position) + self.Kd * (state_desired.velocity - state.velocity)
        return u


