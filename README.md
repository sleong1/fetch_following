# fetch_following
Repository for 41014 Sensors and Control project to control a fetch robot to follow a human ***That has been TAGGED!***.

# Installations

	sudo apt-get install ros-kinetic-aruco-ros

	sudo apt-get install ros-kinetic-aruco-msgs

# Camera Calibration
You'll need to calibrate your camera before using it. Guidance was found in this website:

http://ros-developer.com/2017/04/23/camera-calibration-with-ros/

If you have more than 1 available camera connected to your device, using the following command in terminal to check what they are:

	ls -ltrh /dev/video*

If it is an USB camera, plug and unplugged the usb camera to check which one it is.

if

	rosrun usb_cam usb_cam_node

does not work, just use: 

	roslaunch fetch_following usb_cam_stream_publisher.launch

with fetch_following being the folder you have the launch files saved in.

To calibrate the camera, you will need to run:

	rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.021 image:=/usb_cam/image_raw camera:=/usb_cam --no-service-check

change the variables depending on your specs. If you're using the grid in the calibration folder, then the specs are the following:

1. --size 8x6 <- this being the number of squares being detected
2. --square 0.021 <- this being the size of the squares in meters.

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

**Extra:**

To check out where the aruco-ros installation location:

	roscd aruco_ros

More instructions: (testing in progress)
1)git clone https://github.com/pal-robotics/aruco_ros.git
2)make it
3)add the correct path i.e  
source catkin_ws/devel/setup.sh
export ROS_PACKAGE_PATH=/home/behnam/catkin_ws:$ROS_PACKAGE_PATH
4)roslaunch aruco_marker_finder.launch
