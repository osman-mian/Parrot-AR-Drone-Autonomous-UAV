'''
Created on Mar 16, 2015

@author: sonyc
'''
import numpy as np

class SlidingWindow:
    
    def __init__(self,windowWidth,windowHeight,img,horOffset,verOffset):
        self.windowWidth = windowWidth
        self.windowHeight = windowHeight
        self.img = img
        self.horOffset = horOffset
        self.verOffset = verOffset
        self.x = 0
        self.y = 0
        self.isFull = False
        
    def getNextFrame(self):
        global output
        output = None
        if self.isFull == False:
            output = self.img[self.y:self.y+self.windowHeight-1,self.x:self.x+self.windowWidth-1    ]
            self.updateWindow()
        return output
            
    def updateWindow(self):
        self.x = self.x + self.windowWidth - self.horOffset;
        #print "x %d, y %d, windowWidth %d, height %d" % (self.x,self.y,self.windowWidth, self.windowHeight)
        height,width = self.img.shape
        if((self.x + self.windowWidth-1) >= width):
            self.y = self.y + self.windowHeight - self.verOffset
            if((self.y + self.windowHeight-1) < height):
                self.x = 0
            else:
                self.isFull = True
                
    def getWindows(self):
        w = self.windowWidth - self.horOffset
        h = self.windowHeight - self.verOffset
        hei,wid = self.img.shape
        hei = hei - self.verOffset
        wid = wid - self.horOffset
        #print "Wid is" + w + "Hei is " + h + "Wid isss " + wid + "Hei isss " + hei
        print "W {0}, H {1} , Wid {2}, Hei {3}".format(w,h,wid,hei)
        return (wid//w, hei//h)