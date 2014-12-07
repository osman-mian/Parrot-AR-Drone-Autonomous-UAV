import numpy as np

######################################################
#	Class State
#		Contains an array to store displacement
#		Another array to store velocity
######################################################
class State:
    def __init__(self):
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))




######################################################
#	Class UserCode
#		Proportional Gains are the driving force
#		Derivative Gains are the braking force
######################################################


class UserCode:
    def __init__(self):
        Kp_xy = 0.6										#Proportional Gain X and Y axis
        Kd_xy = 0.4										#Derivative Gain Xand Y axis

        Kp_z = 0.4										#Proportional Gain Z axis
        Kd_z = 0.2										#Derivative Gain Z axis
        
        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T


	##############################################################################
	#	Function 
	#		Computes Command For PD Control
	#		Take Difference of Displacement and multiply it by Proportional gain
	#		Take Difference of Velocities and multiply it by Derivative gain
	##############################################################################
    
    def compute_control_command(self, state, state_desired):
        u = self.Kp * (state_desired.position - state.position) + self.Kd * (state_desired.velocity - state.velocity)
        return u


