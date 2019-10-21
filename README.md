# fetch_following
Repository for 41014 Sensors and Control project to control a fetch robot to follow a human ***That has been TAGGED!***.

# Quick run
To run this on the robot, run the following commands from your kinetic computer, replacing the ip placeholders with your appropriate ip.
```
ROS_MASTER_URI=http://robot.ip:11311
ROS_IP=your.ip
roslaunch fetch_following fetch_following.launch
```
This will launch the aruco ROS package [subscribed to the robot's RGB camera topic] and our motion package.
**Note: Aruco is only supported for indigo and kinetic ROS distributions**

# Fast Travel
1. [Prerequisites](https://github.com/sleong1/fetch_following#prerequisites)
2. [Camera Calibration](https://github.com/sleong1/fetch_following#camera-calibration)
3. [Finding the ARTags with Aruco](https://github.com/sleong1/fetch_following#finding-the-artags-with-aruco)
4. [To run the fetch motion control](https://github.com/sleong1/fetch_following#to-run-the-fetch-motion-control)

# Prerequisites

    aruco-ros
    aruco-msgs

# Camera Calibration
If you are uing Aruco with the simulation, you'll need to calibrate your camera before using it. 
You can also follow the quick steps below:
1. To launch the camera, run: 
```
roslaunch fetch_following usb_cam_stream_publisher.launch
```
To calibrate the camera, you will need to run:
```
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.021 image:=/usb_cam/image_raw camera:=/usb_cam --no-service-check
```
    1. --size 8x6 <- this being the number of squares being detected
    2. --square 0.021 <- this being the size of the squares in meters.

Or you can change them directly in the launch file. Take enough angles till the 'calibrate' button lights up and when you're think you have enough samples. Click the 'calibrate button'. ***Note:*** Be patient. This may take a while and the time of progressing is dependent on your machine. Once it's done, click 'save' to save. If you want your camera to use the same calibration settings, click 'commit'. Good! now you have a yaml file that has all your calibrations saved in.

# Finding the ARTags with Aruco
```
    roslaunch fetch_following aruco_marker_finder.launch markerId:=701 markerSize:=0.1
```
Change `markerId:=701 markerSize:=0.1` to match the specs of your artag or change this directly in the launch. For this project, it has already been changed.

Run 
```
    rosrun rqt_gui rqt_gui
```
to view the output. If you see a blank screen, do not fear, help is here:

1. Plugins > Visualization > Image View
2. Then select your topic

The topic where we will see the pose is in: 

    /aruco_single/result

Generating more Tags
---
To generate the tag, go to: http://chev.me/arucogen/

***This is important.*** **Aruco has a library of saved IDs. It will need an ID from this library to know what ID it is.**

# To run the fetch motion control
This function can be run as a standalone script.
```bash
rosrun fetch_following motion.py
```
