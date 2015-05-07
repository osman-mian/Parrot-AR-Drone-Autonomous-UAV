import numpy as np
from controller import State

class DEqueue:

	def __init__(self,size=5):
		self.buff = size
		self.size = 0
		self.front = 0
		self.Waypoint =[]
		for x in range(0,self.buff):
			self.Waypoint.append(State())
#		for x in range(0,self.buff):
#			print self.Waypoint[x].position

	def addWaypointFront(self,x,y,z):

		size = self.size
		buff = self.buff
		front = self.front
		
		if size==buff:
			return False
		else:
			self.front = (front - 1 + buff) % buff
			self.size= size+1
			self.Waypoint[self.front]=State(x,y,z)
			return True
	def dump(self):
		print "Dumping : "
		for i in range(0,self.buff):
			print self.Waypoint[i].position.T
	

	def addWaypointBack(self,x,y,z):

		size = self.size
		buff = self.buff
		rear = (self.front+size) % buff
		
		if size==buff:
			return False
		else:
			self.size= size+1
			self.Waypoint[rear]=State(x,y,z)
			return True

	def removeWaypointFront(self):
		self.size=self.size-1
		returnable = self.Waypoint[self.front]
		self.Waypoint[self.front]=State()
		
		self.front = (self.front+1)% self.buff
		return returnable

	def removeWaypointBack(self):
		self.size=self.size-1
		rear = (self.front+self.size) % self.buff
		
		returnable = self.Waypoint[rear]
		self.Waypoint[rear]=State()
		return  returnable

	def currentTarget(self):
		returnable = self.Waypoint[self.front]
		return returnable


	def hasWaypoint(self):
		return self.size > 0

	
'''
def haha(a):
	while a.hasWaypoint():
		a.removeWaypointFront()

def main():
	a=DEqueue()
	
	a.addWaypointBack(1000,0,0)
	a.addWaypointBack(2000,0,0)
	a.addWaypointBack(3000,0,0)
	a.addWaypointFront(500,0,0)
	a.dump()
	while a.hasWaypoint():
		print "Discarding" + str(a.currentTarget().T)
		a.removeWaypointFront()


	print 'Done'
	
main()
'''
