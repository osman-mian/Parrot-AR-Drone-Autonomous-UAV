import numpy as np
import math as Math
def calculate_roll_pose(roll_angle):

	roll_angle = roll_angle * (Math.pi/180.0)
	roll_mat = np.array([[1,0,0], [0,Math.cos(roll_angle),Math.sin(roll_angle)],[0,-Math.sin(roll_angle),Math.cos(roll_angle)]])
	
	return roll_mat


def calculate_pitch_pose(pitch_angle):

	pitch_angle = pitch_angle * (Math.pi/180.0)
	pitch_mat = np.array([[Math.cos(pitch_angle),0,Math.sin(pitch_angle)], [0,1,0],[-Math.sin(pitch_angle),0,Math.cos(pitch_angle)]])
	
	return pitch_mat


def calculate_yaw_pose(yaw_angle):

	yaw_angle = yaw_angle * (Math.pi/180.0)
	yaw_mat = np.array([[Math.cos(yaw_angle),Math.sin(yaw_angle),0], [-Math.sin(yaw_angle),Math.cos(yaw_angle),0],[0,0,1]])
	
	return yaw_mat
