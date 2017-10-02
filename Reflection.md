# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Project Goal :
The goal in this project is to build a path planner that creates smooth, safe trajectories for the car to follow.
a) Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
b) The car should try to go as close as possible to the 50 MPH speed limit.
c) Other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
d) The car should be able to make one complete loop around the 6946m highway.
e) Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
f) The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Project Details :
Below is the brief detail how solution works:
a) Car drive in lane to reach the goal and maximum speed it can be go upto 49.5 mph. I took 
b) If there is other car in lane and blocking the movement of car, then car will see if there is available lane to change. If yes, then car will keep the speed and change the lane.
c) If there is no other lane available for change, car will slow down its speed in current lane to avoid collision.

## Different inputs:
Below are list of the input available for this projects.
a) highway_map.csv, This consist of different waypoints on the highway. It has x,y,s,dx and dy value for each waypoints.
b) we have sensors_fusion data which consist of data for other cars on highway.It has carids and then values such as x,y,vx,vy,s,d for that car. Its two dimesnional vectors
c) Localization data for ego car. it consist of car_x, car_y, car_s, car_d, car_yaw, speed
d) Similator also provide the previous path data as well. This would help for smoth transitions. 

## Project Directory structures :

Below is the directory structure of the projects.
a)  data directory which consist of highway_map.csv
b) src/
* main.cpp : Main file which send the updated x,y value to simulator to drive the car on highway 
* Helper.cpp and Helper.h : There are lots of helper function which provided along the project . I included everything in one class , so other files/class can use them easily. Also added additional function to print Debug Info on the console window.
* PathPlanner.cpp and PathPlanner.h : This class is main responsible of path planning . Driving the car safely and doing the lane change without collision.
* spline.h : In order to estimate the location of points between the known waypoints, we need to "interpolate" the position of those points. Spilne can be used for this task. It is simple to use and requiring no dependencies.
