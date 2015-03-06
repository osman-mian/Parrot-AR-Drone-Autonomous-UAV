import controller as pd
import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')

from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import numpy as np

from Queue import Queue


import pose_matrix as poses
import average_window
import math

from DEqueue import DEqueue
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
target.position = np.array([[1000],[0],[0]])
target.velocity =  np.array([[0],[0],[0]])


##############################################

#pub_velocity = rospy.Publisher('/cmd_vel', Twist)
#pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
#pub_land = rospy.Publisher('/ardrone/land', Empty)
#pub_reset = rospy.Publisher('/ardrone/reset', Empty)


window=5
v_q = Queue(window)
yaw_correction =0

path_queue = 0# DEqueue()
############################################
#This function is called, everytime a message containing Navigation Data is received from drone
def callback(navdata):

#	pub_land = rospy.Publisher('/ardrone/land', Empty)	#This is the publisher to issue the landing command to the drone
	global path_queue

	if path_queue.hasWaypoint():
		goToNextWayPoint(path_queue,navdata)
	else :
#		rospy.sleep(1)
		print "Last station"
		global flag
		flag=True
#		pub_land.publish(Empty())

def  goToNextWayPoint(path_queue,navdata):
	global t
	global st
	global pub_velocity
	global code
	global position_local
	global target
	global v_q
	global yaw_correction
	global flag
	#compute time since last call
	#t2=t
	#t = navdata.header.stamp.to_sec()
	dt= 1/155.0

	if v_q.full():

		v_q.get()
		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))

		roll_mat = poses.calculate_roll_pose(navdata.rotX);
		pitch_mat = poses.calculate_pitch_pose(navdata.rotY);
		yaw_mat = poses.calculate_yaw_pose(navdata.rotZ+yaw_correction);
		
		pose=np.dot(np.dot(roll_mat,pitch_mat),yaw_mat);
#		print(pose)
		#get local distance co-ordinates
		position_local= dt *  average_window.getAverage(v_q,window)
		#print(st.position)
		#print(target.position)

		#print "pose"
		#print yaw_mat


		#print "dot prod"
		#print np.dot(pose,position_local)

		st.position = st.position + np.dot(pose,position_local)
		st.velocity = np.dot(pose,average_window.getAverage(v_q,window))


		#print "local_position"
#		print("----Global-----")
#		print st.position

#		print("******Command******")		

		#compute PD
		u =	code.compute_control_command(st,path_queue.currentTarget());
	
	
		#reconvert to local
		local_command= np.dot(pose.T,u)
	

		#print(dt)
#		print(str(st.velocity.T[0,0])+", "+str(st.velocity.T[0,1])+", "+str(st.velocity.T[0,2]))
		print(str(st.position.T[0,0])+", "+str(st.position.T[0,1])+", "+str(st.position.T[0,2]))
		co=1
		if(reachedTarget(path_queue.currentTarget())) :
			t=1#
			print "Checkpoint" + str(co)
			path_queue.removeWaypointFront()
			flag=False;
			co=co+1
			print(st.velocity)


#		print(local_command);
		navdata.vx=local_command.T[0,0]
		navdata.vy=local_command.T[0,1]
		navdata.vz=local_command.T[0,2]

		#
	else:
		yaw_correction= 0#navdata.rotZ * -1
		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))

########################################	



class Nav:
	def __init__(self):
		self.rotX=0;
		self.rotY=0;
		self.rotZ=0;
		self.vx=0;
		self.vy=0;
		self.vz=0;





###########################
flag=False

def reachedTarget(target):
	global flag
	global st
#	global target

	temp = st.position - target.position
	temp_sp=st.velocity - target.velocity
	
	flag_dist=math.fabs(temp.T[0,2])< 2 and math.fabs(temp.T[0,1]) < 2 and math.fabs(temp.T[0,0]) < 2
	flag_sp=math.fabs(temp_sp.T[0,2])< 1 and math.fabs(temp_sp.T[0,1]) < 1 and math.fabs(temp_sp.T[0,0]) < 1

	#print(temp)

	if  flag_dist==True and flag_sp == True:
		flag=True
		st.velocity=np.array([[0], [0], [0]])

		
		return True
	
	return False

def main():
	global flag;
	global path_queue
	path_queue= DEqueue()
	path_queue.addWaypointBack(20,0,0)
	path_queue.addWaypointBack(50,0,0)
	#rospy.init_node('example_node', anonymous=True)
	#rospy.Subscriber("/ardrone/navdata", Navdata, callback)

	navy= Nav()
	i=0;
	while(flag!=True):
		callback(navy)
		i=i+1



main()



