import controller as pd
import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')

from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import numpy as np

t=0
st=pd.State();
pub_velocity = rospy.Publisher('/cmd_vel', Twist)
pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
pub_land = rospy.Publisher('/ardrone/land', Empty)
pub_reset = rospy.Publisher('/ardrone/reset', Empty)

def callback(navdata):
	global t
	global st
	global pub_velocity

	st.velocity=np.array([[navdata.vx],[navdata.vy],[navdata.vz]])

	t2=t
	t = navdata.header.stamp.to_sec()
	dt= t-t2

	st.position= dt * st.velocity
	print(st.position)

	print("--------------")
	

def main():

	rospy.init_node('example_node', anonymous=True)

	code=pd.UserCode();

	rospy.Subscriber("/ardrone/navdata", Navdata, callback)

	

	print("aha")
	rospy.spin()

	print("Done")
'''
	while(1)
		st.velocity= chopper.getVel();
		colVector3 dist= vel * dt;
		st.position = pose * dist

		u =		UserCode.computeControllCommand(global_cordinate,desired_cordinate,vel,desired_vel);
	
		cmdvel.publish(u);

		
		

'''



main()
