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
target.position = np.array([[1500],[-1500],[0]])
target.velocity =  np.array([[0],[0],[0]])


##############################################

pub_velocity = rospy.Publisher('/cmd_vel', Twist)

obstruction=False
window=5
v_q = Queue(window)
wait =0

yaw_adjust=0;
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
	global yaw_adjust

	#compute time since last call
	t2=t
	t = navdata.header.stamp.to_sec()
	dt= t-t2

	pub_land = rospy.Publisher('/ardrone/land', Empty)

	if v_q.full():

		#increment step count
		i=i+1 

		#add an element to queue
		v_q.get()
		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))

		
		#print("YAW: "+str(navdata.rotZ) )
		#print("Real: "+str(navdata.rotZ+yaw_adjust))

		#roll_mat = poses.calculate_roll_pose(navdata.rotX);
		#pitch_mat = poses.calculate_pitch_pose(navdata.rotY);
		#yaw_mat = poses.calculate_yaw_pose(navdata.rotZ + yaw_adjust);

		#print(yaw_mat)

		pose=np.dot(np.dot(roll_mat,pitch_mat),yaw_mat);

		#get local distance co-ordinates
		position_local= dt *  average_window.getAverage(v_q,window) #np.array([[navdata.vx],[navdata.vy],[navdata.vz]])
		#print(st.position)
	

		st.position = st.position + (np.dot(pose,position_local))
		st.velocity = np.dot(pose,average_window.getAverage(v_q,window))

		#print(pose)
		#print("%%%%%%%%%%%%")
		#print(navdata.vz)	

		#print(st.position)

		#compute PD
		u =	code.compute_control_command(st,target);
	
	
		#reconvert to local
		local_command= np.dot(pose.T,u)
	
		

		if obstruction == False :
#			print(str(local_command.T[0,0]/1000.0)+", "+str(local_command.T[0,1]/1000.0)+", "+str(local_command.T[0,2]/1000.0))
			print(str(st.position.T[0,0])+", "+str(st.position.T[0,1])+", "+str(st.position.T[0,2]))
			wait=0
			pub_velocity.publish(Twist(Vector3(local_command.T[0,0]/1000.0,local_command.T[0,1]/1000.0,local_command.T[0,2]/1000.0),Vector3(0,0,0)))
		else :
			pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
			print("Obstruction Detected")
			wait = wait+1

		#pub_velocity.publish(local_command);

		print("--------------")
		if wait > 500 or reachedTarget() or i>2000:
			pub_land.publish(Empty())
			if i>2000:
				print ("Time Limit Exceeded")
			elif wait > 300:
				print("Too big obstruction")
			else:
				print("Target Reached")

	else:
		yaw_adjust = -navdata.rotZ
		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))

	
########################################	
def reachedTarget():
#	global flag
	global st
	global target

	temp = st.position - target.position
	temp_sp=st.velocity - target.velocity
	
	dist_error=130
	sp_error=40

	flag_dist=math.fabs(temp.T[0,2])< dist_error and math.fabs(temp.T[0,1]) < dist_error and math.fabs(temp.T[0,0]) < dist_error
	flag_sp=math.fabs(temp_sp.T[0,2])< sp_error and math.fabs(temp_sp.T[0,1]) < sp_error and math.fabs(temp_sp.T[0,0]) < sp_error
#	print("DONE:")

#	print("Speed: "+str(temp_sp.T[0,0])+", "+str(temp_sp.T[0,1])+", "+str(temp_sp.T[0,2]))
#	print("distance: "+str(temp.T[0,0])+", "+str(temp.T[0,1])+", "+str(temp.T[0,2]))
	#print(temp)

	if  flag_dist==True and flag_sp == True:
#		flag=True
		print("DONE:")
		return True
	
	return False
########################################

def callback2(sonar):

	global obstruction
	z=str(sonar).split()[1];

	if int(z) < 20:
		obstruction =True
		#print ("Cant Move")
	else:
		obstruction =False
		#print ("Moving Again")

#########################################
def main():

	a=10;
	rospy.init_node('example_node', anonymous=True)
    
    # publish commands (send to quadrotor)
	pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
	pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    
	print("ready!")
	rospy.sleep(1.0)
    
	print("takeoff..")
	pub_takeoff.publish(Empty())
	rospy.sleep(4.0)

	#input('ready?')

	rospy.Subscriber("/ardrone/navdata", Navdata, callback)
#	rospy.Subscriber("/Distance",Int32, callback2)

	rospy.spin()

#	pub_land.publish(Empty())

	print("Fine!")

main()
