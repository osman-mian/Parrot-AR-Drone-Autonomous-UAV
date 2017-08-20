# Final Year Project 
### Bachelor of Science: Computer Science

#### Title: Autonomous UAV

**DESCRIPTION**

The aim of the project was to make an off the shelf drone (PARROT AR 2) drone autonomous such that
* It Can Fly From A Given Point A to Another Given Point B 
* Not Hit Any Obstacles In The Process 
* Identify a “Safe Spot” and Park Itself There 

A playlist containing videos of the experiments conducted is available [here](https://www.youtube.com/playlist?list=PLiXKVHYiGb40jBA6rYrenqKA5pRNH5zRB)

**Technology Stack**
* Robot Operating System
* Python Programming Language
* Scikit Python
* Open CV Python
* RaspberryPi Programming


**Brief Introduction**

We came up with a prototype of the drone that is capable of taking off at an unknown
location, explore a certain number of randomly generated co-ordinates, and analyze the visual feeds at these co-ordinates to decide a safe spot and land there. For the scope of this project, doors and windows qualify as unsafe spots. The drone is also capable of avoiding any obstacles that come in the direction of its front camera. 

We accomplished these taks by writing two separate modules one each for object recognition/detection and for autonomous navigation. 

1. The autonomous navigation module comprised of two subsystems, one for developing the PD control to allow the drone to go from a given point A to point B, second one for mounting a sonar sensor prototype on the drone to make it avoid colliding into obstacles in its direction of heading. 

2. The classification subsystem was created using the HOG features trained over an SVM using an RBF kernel. We used a total of 2 models (one with indoor pictures and one with outdoor pictures), their results can be seen in prototype details of the technical report available in this repository.


