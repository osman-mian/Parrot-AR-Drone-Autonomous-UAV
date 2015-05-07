import slidingWindow as sw
import numpy as np
import cv2

img = cv2.imread("poster.jpg",0)
test = sw.SlidingWindow(128,256,img,32,64)

imag = np.array([])
whileLoop = True

#for i in range(0,50):
test.getWindows()
'''
while imag != None:
    imag = test.getNextFrame()
    print imag.size
    cv2.imshow("Hello",imag)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    ''' 