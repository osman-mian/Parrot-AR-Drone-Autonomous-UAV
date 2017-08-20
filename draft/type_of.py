import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')



#AR DRone related libraries
from ardrone_autonomy.msg import Navdata
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from checker import image_converter

import numpy as np


flag= False;

x=image_converter();
count=0;

def picBack(raw_image):
	global count
	
	#print(count)
	if count==100:
		x.isSafe(raw_image)
		count=0
	else:
		count=count+1
	



def main():
	
	rospy.init_node('example_node', anonymous=True)
	rospy.Subscriber("/ardrone/image_raw", Image, picBack)
	rospy.spin()
	
	

	
main()
