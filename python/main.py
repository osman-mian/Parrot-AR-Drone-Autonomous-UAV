
#ROS related Libraries
import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')


#AR DRone related libraries
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import numpy as np


#Python's Native Libraries
from Queue import Queue
import math
import random



#Our Own Written Python Files
import pose_matrix as poses
import average_window
import controller as pd
from DEqueue import DEqueue

from image_processing import image_converter
from sonar import Distance_Sensor

################################################


t=0					#initial time
code=pd.UserCode()	#object that produces next command for PD control


#ORIENTATION matrices
roll_mat = np.array([[1,0,0], [0,1,0],[0,0,1]])			#Rotation around X-Axis
pitch_mat = np.array([[1,0,0], [0,1,0],[0,0,1]])		#Rotation around Y-Axis
yaw_mat = np.array([[1,0,0], [0,1,0],[0,0,1]])			#Rotation around Z-Axis

###############################################


#State Variable contains position and speed vectors
st=pd.State()

st.position = np.array([[0],[0],[0]])					#Instantaneous position in global frame
st.velocity =  np.array([[0],[0],[0]])					#Instantaneous Velocity in Global Frame


position_local = np.array([[0],[0],[0]])				#Stores the product of distance * time in local coordinate frame



target=pd.State()
target.position = np.array([[1000],[0],[0]])		#Desired position in global frame		
target.velocity =  np.array([[0],[0],[0]])				#Desired velocity in global frame


############## GLOBAL VARIABLES ###############

pub_velocity = rospy.Publisher('/cmd_vel', Twist)		#Publishing Object, to issue velocity commands to the drone

picture = True											#Bit is set when picture is to be taken
obstruction=False										#Shared Flag set/reset using sonic sensor to inform drone of obstacles
window=5												#used to sum up last N readings and take their average, to smooth out current speed
v_q = Queue(window)										#Queue which stores last 5 speed readings, so that we can take their average
wait =0													#Wait time, since obstruction has been detected

yaw_adjust=0;											#Equal to negative of initial yaw angle, make sure init bearing is 0 degrees
i=0;													#Pre Defined Max Iterations in case the drone behaves unexpectedly

path_queue = DEqueue()
safe_house = DEqueue()
													


ic=0
sonar=0
############################################


#This function is called, everytime a message containing Navigation Data is received from drone
def callback(navdata):

	pub_land = rospy.Publisher('/ardrone/land', Empty)	#This is the publisher to issue the landing command to the drone
	global path_queue

	if path_queue.hasWaypoint():
		goToNextWayPoint(path_queue,navdata)
	else :
		print "Last Destination"
		rospy.sleep(1)
		pub_land.publish(Empty())
		print "Safe Spots"
		safe_house.dump()

vz=0
def goToNextWayPoint(path_queue,navdata):
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
	global picture
	global ic
	global vz

	


	#########################################
	#	Step1: compute time since last call #
	#########################################
	t2=t
	t = navdata.header.stamp.to_sec()
	dt= t-t2

	#############################################################
	#	Step2:
	#		If first 5 frames are completed do the following
	#			Navigate drone to a point
	#			Land if already there
	#			Make Adjustments if obstacles is detected
	#		Else
	#			Add current velocity reading to our queue
	#############################################################

	if v_q.full():

		
		i=i+1 																	#increment step count

		
		v_q.get()
#		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))				#add new velocity to our Queue
		v_q.put(np.array([[navdata.vx],[navdata.vy],[vz]]))				#add new velocity to our Queue
		
		#print("YAW: "+str(navdata.rotZ) )
		#print("Real: "+str(navdata.rotZ+yaw_adjust))

		roll_mat = poses.calculate_roll_pose(navdata.rotX);						#Calculate Drones Roll Angle (Gives erroneous readings)
		pitch_mat = poses.calculate_pitch_pose(navdata.rotY);					#Calculate Drones Pitch Angle (Gives erroneous readings)

		yaw_mat = poses.calculate_yaw_pose(navdata.rotZ + yaw_adjust);			#Calculate Current Yaw Angle (The Bearing)

		pose=np.dot(np.dot(roll_mat,pitch_mat),yaw_mat);						#Concatenate these matrices to obtain pose in global frame


		position_local= dt *  average_window.getAverage(v_q,window) 			#Compute Distance travelled in local frame


		st.position = st.position + (np.dot(pose,position_local))				# P_global = Pose * P_local, get the distance travelled globally
		st.velocity = np.dot(pose,average_window.getAverage(v_q,window))		# Get the velocity in global frame



		u =	code.compute_control_command(st,path_queue.currentTarget());							#Use target and current positions to compute next command
	
		local_command= np.dot(pose.T,u)											# P_local = Pose_inv * P_global
		
		if (not sonar.proximity_warning()):												#If no obstruction issue the new commands
			#print(str(st.position.T[0,0])+", "+str(st.position.T[0,1])+", "+str(st.position.T[0,2]))
			wait=0
			pub_velocity.publish(Twist(Vector3(local_command.T[0,0]/1000.0,local_command.T[0,1]/1000.0,local_command.T[0,2]/1000.0),Vector3(0,0,0)))
			vz=local_command.T[0,2]
		else :																	#Displace randomly in 1 direction otherwise
			
			print("")

			'''
			z=randrange(1, 3)

			if z==1:
				pub_velocity.publish(Twist(Vector3(0,-1,0),Vector3(0,0,0)))
			elif z==2:
				pub_velocity.publish(Twist(Vector3(0,1,0),Vector3(0,0,0)))
				
			wait = wait+1
			rospy.sleep(0.5)
			'''
			pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))


#		print("--------------")
		
		############################################################
		#Stop the Drone If: 
		#		Obstruction was detected for longer than 500 frames 
		#		OR
		#		Destination Co-ordinate has been reached
		#		OR
		#		A Predefined Maximum Iteration limit has been reached
		############################################################
		if wait > 500 or reachedTarget(path_queue.currentTarget()) :#or i>2000:
			pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
			
			#if i>2000:
			#	print ("Time Limit Exceeded")
			if wait > 500:
				print("Too big obstruction")
			else:
				print("Checkpoint Reached")
				ic.ready=True
				picBack()
				path_queue.removeWaypointFront()
				pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
				rospy.sleep(3)
				
				

	else:
		yaw_adjust = -navdata.rotZ
		v_q.put(np.array([[navdata.vx],[navdata.vy],[navdata.vz]]))

	



####################################################################################
# Function to Detect if Destination position has been reached
#
# 	Find Manhatan Distance between current and desired coordinates
#	Find Manhatan Distance between current and desired velocities
# 	If Both Distances are within Error Limit
#		Target Has been reached
#	Else
#		Target Not Yet Reached
####################################################################################

def reachedTarget(target):

	global st
	global picture

	temp = st.position - target.position							#Calculate Distance from Target Position,
	temp_sp=st.velocity - target.velocity								#Calculate Difference from Target Velocity
	
	dist_error=130														#Assume Target reached if global position is within this limit
	sp_error=40															#Assume Target reached if global velocity is within this limit

	flag_dist=math.fabs(temp.T[0,2])< dist_error and math.fabs(temp.T[0,1]) < dist_error and math.fabs(temp.T[0,0]) < dist_error
	flag_sp=math.fabs(temp_sp.T[0,2])< sp_error and math.fabs(temp_sp.T[0,1]) < sp_error and math.fabs(temp_sp.T[0,0]) < sp_error


	#if Both Displacement and speed are within error limits, return true. False Otherwise
	if  flag_dist==True and flag_sp == True:
		return True
	
	return False






####################################################################################
#Function to process information received by camera sensor, 
#		
#	If picture_flag is on
#		save image
# 	Else
#		do nothing
####################################################################################
def picBack():
	

	global path_queue
	global safe_house;
	
#	if ic.isSafe():
#		safe_house.addWaypointBack(path_queue.currentTarget())


	
#######################################################
#	Main Function
#
#		Initialize the ROS client
#		Take Off The Drone
#		Subscribe to Navigation Data Updates
#		Subscribe to Sonar Sensor Updates
#		Initialize a spin lock until program exits
#######################################################

def main():
	global path_queue
	global ic
	global sonar
	path_queue.addWaypointBack(0,0,0)
	path_queue.addWaypointBack(1000,0,0)
	#path_queue.addWaypointBack(1000,-500,0)
	#path_queue.addWaypointBack(0,-500,0)
	path_queue.addWaypointBack(0,0,0)

	a=10;
	rospy.init_node('example_node', anonymous=True)
    
    # publish commands (send to quadrotor)
	pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
	pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    
	print("ready!")
	#pub_reset.publish(Empty())
	rospy.sleep(2.0)
    
	print("takeoff..")
	#pub_takeoff.publish(Empty())
	rospy.sleep(2.0)

	ic = image_converter()
	sonar = Distance_Sensor()

	print("Subscribing")
	rospy.Subscriber("/ardrone/navdata", Navdata, callback)
	#rospy.Subscriber("/ardrone/image_raw", Image, picBack)
	#rospy.Subscriber("/chatter",Int32, callback2)

	rospy.spin()


main()
