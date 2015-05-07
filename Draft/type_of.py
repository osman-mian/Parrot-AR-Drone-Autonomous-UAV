import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')



#AR DRone related libraries
from ardrone_autonomy.msg import Navdata
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
import numpy as np


flag= False;


def picBack(raw_image):
	print(type(raw_image.vx)n
	);



def main():
	
	rospy.init_node('example_node', anonymous=True)
	rospy.Subscriber("/ardrone/navdata", Navdata, picBack)
	rospy.spin()
	
	

	
main()
