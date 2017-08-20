import rospy
import roslib
from std_msgs.msg import Int32
import numpy as np

####################################################################################
#Function to process information received by sonar sensor, 	Measurments are in CM
#		
#	If distance of object from direction faced is less than 40 cm
#		Indicate Obstruction
# 	Else
#		Indicate No Obstruction
####################################################################################
class Distance_Sensor:

	def __init__(self):
		self.distance=100;
		self.sonar_sub = rospy.Subscriber("/chatter",Int32,self.callback)
		self.threshold = 30


	def callback(self,data):
		z=str(data).split(' ')
		self.distance=int(z[1])
		if(self.proximity_warning()):
			print "Danger"
		

	def proximity_warning(self):
		#print str(self.distance) + " " + str(self.threshold)
		
		if self.distance < self.threshold:
			return True
		return False
