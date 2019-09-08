# fetch_following
Repository for 41014 Sensors and Control project to control a fetch robot to follow a human ***That has been TAGGED!***.

# Installations

	sudo apt-get install ros-kinetic-aruco-ros

	sudo apt-get install ros-kinetic-aruco-msgs

# Camera Calibration
You'll need to calibrate your camera before using it. Guidance was found in this website:
http://ros-developer.com/2017/04/23/camera-calibration-with-ros/

# General reminders (mainly for myself =3)
A Dummy's Guide to things
---
Before running anything in a freshly born terminal, remember to source!
---
	cd catkin_ws
	source devel/setup.bash

and if that doesn't work, perhaps try:

	catkin_make

then try running whatever you're running again.

Getting output fom usb camera
---
To run get publish images from the usb camera AND see the output immediately:

	roslaunch usb_cam usb_cam-test.launch

Followed aruco instructions from here:
---
http://ros-developer.com/2017/04/23/aruco-ros/

Or follow these steps:
Step 1. Open a new terminal and source it!

	source devel/setup.bash

Step 2. launch the aruco:

	roslaunch aruco_marker_finder.launch markerId:=701 markerSize:=0.05

Extra:
To check out where the aruco-ros installation location:

	roscd aruco_ros


