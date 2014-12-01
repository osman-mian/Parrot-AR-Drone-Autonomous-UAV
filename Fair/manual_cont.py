import controller as pd
import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')

from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import numpy as np


import pose_matrix as poses
import math

################################################
t=0
code=pd.UserCode()


#ORIENTATION
roll_mat = np.array([[1,0,0], [0,1,0],[0,0,1]])
pitch_mat = np.array([[1,0,0], [0,1,0],[0,0,1]])
yaw_mat = np.array([[1,0,0], [0,1,0],[0,0,1]])
###############################################


#start state
st=pd.State()
st.position = np.array([[0],[0],[0]])
st.velocity =  np.array([[0],[0],[0]])


position_local = np.array([[0],[0],[0]])


#target state
target=pd.State()
target.position = np.array([[-40],[10],[2]])
target.velocity =  np.array([[0],[0],[0]])


##############################################

pub_velocity = rospy.Publisher('/cmd_vel', Twist)
pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
pub_land = rospy.Publisher('/ardrone/land', Empty)
pub_reset = rospy.Publisher('/ardrone/reset', Empty)


############################################

def callback(navdata):
	global t
	global st
	global pub_velocity
	global code
	global position_local
	global target
	#compute time since last call
	#t2=t
	#t = navdata.header.stamp.to_sec()
	dt= 1/15.0;


	roll_mat = poses.calculate_roll_pose(navdata.rotX);
	pitch_mat = poses.calculate_pitch_pose(navdata.rotY);
	yaw_mat = poses.calculate_yaw_pose(navdata.rotZ);

	pose=np.dot(np.dot(roll_mat,pitch_mat),yaw_mat);

	#get local distance co-ordinates
	position_local= dt * np.array([[navdata.vx],[navdata.vy],[navdata.vz]])
	#print(st.position)

	print "pose"
	print yaw_mat


	#print "dot prod"
	#print np.dot(pose,position_local)

	st.position = st.position + np.dot(pose,position_local)
	st.velocity = np.dot(pose,np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))


	print "local_position"
	print st.position

#	print st.position.shape

#	print np.dot(pose,position_local).shape

	
	

	

	#compute PD
	u =	code.compute_control_command(st,target);
	
	
	#reconvert to local
	
	local_command= np.dot(pose.T,u)
	

	#print(dt)
	#print(local_command)
	#cmdvel.publish(local_command);
	navdata.vx=local_command.T[0,0]
	navdata.vy=local_command.T[0,1]
	navdata.vz=local_command.T[0,2]

	print("--------------")

########################################	



class Nav:
	def __init__(self):
		self.rotX=0;
		self.rotY=0;
		self.rotZ=-90;
		self.vx=0;
		self.vy=0;
		self.vz=0;





###########################


def main():

	#rospy.init_node('example_node', anonymous=True)
	#rospy.Subscriber("/ardrone/navdata", Navdata, callback)

	navy= Nav()
	i=0;
	while(i<15*8):
		callback(navy)
		i=i+1



main()



