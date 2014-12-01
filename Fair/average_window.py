import numpy as np
from Queue import LifoQueue

def getAverage(q,size):

	sum_=np.array([[0],[0],[0]]	)

	for num in range(0,size):
		temp=q.get()
		sum_ = sum_ + temp
		q.put(temp)

	sum_ = sum_ / size


	return sum_
