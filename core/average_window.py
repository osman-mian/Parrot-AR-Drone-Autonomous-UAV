import numpy as np
from Queue import LifoQueue

##############################################################################
#
#	Function to calculate average velocity over last N number of readings,
#	where N = size
#
#	Steps
#	For each element from 1 to size
#		Deque an element
#		Add its velocity to respective axis-es
#		Enque the element again
#	End For
#
#	Average = sum / size
#
#	Return Average
#
##############################################################################


def getAverage(q,size):

	sum_=np.array([[0],[0],[0]]	)

	for num in range(0,size):
		temp=q.get()
		sum_ = sum_ + temp
		q.put(temp)

	sum_ = sum_ / size


	return sum_
