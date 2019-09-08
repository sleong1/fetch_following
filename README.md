# fetch_following
Repository for 41014 Sensors and Control project to control a fetch robot to follow a human.

# General reminders (mainly for myself =3)
Before running anything, remember to source:

	cd catkin_ws
	source devel/setup.bash

and if that doesn't work, perhaps try:

	catkin_make

then try running whatever you're running again.
---

To run get publish images from the usb camera:

	roslaunch usb_cam usb_cam-test.launch

Followed aruco instructions from here:

http://ros-developer.com/2017/04/23/aruco-ros/

Or follow these steps:

1. go to the directory where your aruco-ros is:

	roscd aruco_ros

2. launch the aruco:

	roslaunch aruco_marker_finder.launch markerId:=701 markerSize:=0.05

# Camera Calibration

http://ros-developer.com/2017/04/23/camera-calibration-with-ros/


# Installations

	sudo apt-get install ros-kinetic-aruco-ros

	sudo apt-get install ros-kinetic-aruco-msgs



