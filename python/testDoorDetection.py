import WindowDetector as wd
import slidingWindow as sw
import numpy as np
import os.path
import cv2

fileName = "something.jpg"
if os.path.isfile(fileName):
    img = cv2.imread(fileName,1)

test = wd.DoorDetector("myModel.xml")
p_label,p_acc,p_val = test.detectDoor(img,32,60)

if p_acc[0] > 0:
    print "Door Detected", p_acc[0]
else:
    print "NO Door FOUND"