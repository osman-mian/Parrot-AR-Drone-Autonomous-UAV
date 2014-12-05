import controller as pd
import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')


from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import numpy as np
from Queue import Queue
import math

import pose_matrix as poses
import average_window

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
target.position = np.array([[10],[0],[0]])
target.velocity =  np.array([[0],[0],[0]])


##############################################

pub_velocity = rospy.Publisher('/cmd_vel', Twist)

obstruction=False
window=5
v_q = Queue(window)
wait =0
############################################
i=0;
def callback(navdata):
	global t
	global st
	global pub_velocity
	global code
	global position_local
	global target
	global v_q
	global i
	global obstruction
	global wait

	#compute time since last call
	t2=t
	t = navdata.header.stamp.to_sec()
	dt= t-t2
	pub_land = rospy.Publisher('/ardrone/land', Empty)

	if v_q.full():
		i=i+1
		v_q.get()
		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))

		#roll_mat = poses.calculate_roll_pose(navdata.rotX);
		#pitch_mat = poses.calculate_pitch_pose(navdata.rotY);
		#yaw_mat = poses.calculate_yaw_pose(navdata.rotZ);
	
		pose=np.dot(np.dot(roll_mat,pitch_mat),yaw_mat);

		#get local distance co-ordinates
		position_local= dt *  average_window.getAverage(v_q,window) #np.array([[navdata.vx],[navdata.vy],[navdata.vz]])
		#print(st.position)
	
		st.position = st.position + (np.dot(pose,position_local))
		st.velocity = np.dot(pose,average_window.getAverage(v_q,window))

		#print	st.position
		#print "-************-\n"
	#	print pose
	

	

		#compute PD
		u =	code.compute_control_command(st,target);
	
	
		#reconvert to local
	
		local_command= np.dot(pose.T,u)
	

		#print(dt)
		

		if obstruction == False :
			print(local_command)
			wait=0
			#pub_velocity.publish(Twist(Vector3(local_command.T[0,0],local_command.T[0,1],local_command.T[0,2]),Vector3(0,0,0)))
		else :
			pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
			print("Obstruction aa gai :( ")
			wait = wait+1

		#pub_velocity.publish(local_command);

		print("--------------")
		if wait > 300 or reachedTarget(local_command.T):
			print("landing")
			#pub_land.publish(Empty())
	else:
		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))

	
########################################	
def reachedTarget(local_command):
	if math.fabs(local_command[0,0]) < 0.2 and math.fabs(local_command[0,1]) < 0.2 :
		return True
	return False

###########
def callback2(sonar):

	global obstruction
	z=str(sonar).split()[1];

	if int(z) < 20:
		obstruction =True
		#print ("Cant Move")
	else:
		obstruction =False
		#print ("Moving Again")


def main():

	a=10;
	rospy.init_node('example_node', anonymous=True)
    
    # publish commands (send to quadrotor)
	pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
	pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    
	print("ready!")
	rospy.sleep(1.0)
    
	#print("takeoff..")
	#pub_takeoff.publish(Empty())
	#rospy.sleep(7.0)

	rospy.Subscriber("/ardrone/navdata", Navdata, callback)
	rospy.Subscriber("/Distance",Int32, callback2)

	rospy.spin()

#	pub_land.publish(Empty())

	print("Fine!")

main()
