
#ROS related Libraries
import rospy
import roslib
import roslib; roslib.load_manifest('ardrone_python')


import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import WindowDetector as wd
import slidingWindow as sw
import numpy as np



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.x=0
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
    self.safety=False
    self.ready=False


  def callback(self,data):

	#every after 100 callbacks... render an image and test it on model
#    if self.x>20: 
    if self.ready:	
	    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    cv2.imshow("Image window", cv_image)
	    cv2.waitKey(3)

	    try:

	          test = wd.DoorDetector("Pos4Neg2(0).xml")
	          img = cv2.resize(cv_image,(640,360))
	          
	          p_label,p_acc,p_val = test.detectDoor(cv_image,0,0)
	          if p_acc[0] > 0:
	          	print "Door Detected", p_acc[0]
	          	self.safety=False
	          else:
	          	print "NO Door FOUND"
	          	self.safety=True
	          	print self.safety
	          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	          self.ready=False
	    except CvBridgeError, e:
	          print e

	    self.x=0
    else:
	    self.x=self.x+1



  def isSafe(self):
    print "Testing " + str(self.safety)
    return self.safety

'''
def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()


'''
