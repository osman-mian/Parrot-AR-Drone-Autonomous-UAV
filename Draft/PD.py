#!/usr/bin/env python
import numpy as nm
import math as Math

if __name__ == '__main__':

	
	fout = open('data.csv', 'w')
	
	P=0.5; 			#proportional gain
	D=0.3; 			#derivative gain

	dt=1; 		#time interval in seconds (assumption)
	counter=1;
	dist=0; 		#current X cordinate
	desired_loc=5; 	#desired X cordinate (randomly chosen)
	v=0;			#current X velocity
	u=0.0			#control command

	#repeat for 50 time step
	while counter < 50 :

		dist+= v * dt;
		u = (P * (desired_loc-dist)) + (D * (0-v)); 
		v= v + (u*dt);
		fout.write(str(dist)+", \n");
		counter+=1;


	print("Done!");
	# x_offset = P * (current_loc - desired_loc) + D * (current_v - desired_v)

