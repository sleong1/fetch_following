# fetch_following
Repository for 41014 Sensors and Control project to control a fetch robot to follow a human ***That has been TAGGED!***.

##### Fast Travel
[Installations](https://github.com/sleong1/fetch_following#installations)
[Camera Calibration](https://github.com/sleong1/fetch_following#camera-calibration)
[General reminders](https://github.com/sleong1/fetch_following#general-reminders-mainly-for-myself-3)
[To Run the Pose detector](https://github.com/sleong1/fetch_following#to-run-the-pose-detector)
[To get the rosbag](https://github.com/sleong1/fetch_following#to-get-the-rosbag)
[To run the simulation](https://github.com/sleong1/fetch_following#to-run-the-simulation)
[To run the fetch motion control](https://github.com/sleong1/fetch_following#to-run-the-fetch-motion-control)
[How to be a Parasite (connecting to a host and sharing a roscore)](https://github.com/sleong1/fetch_following#how-to-be-a-parasite-connecting-to-a-host-and-sharing-a-roscore)

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

Or you can change them directly in the launch file. Take enough angles till the 'calibrate' button lights up and when you're think you have enough samples. 

Click the 'calibrate button'. ***Note:*** Be patient. This may take a while and the time of progressing is dependent on your machine. 

Once it's done, click 'save' to save. If you want your camera to use the same calibration settings, click 'commit'. Good! now you have a yaml file that has all your calibrations saved in.

Extra documention for camera calibration:

http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Camera calibration yaml
---
If you don't want to calibrate and you trust mine, here's the yaml for the usb camera used in this project.

go to:

    cd .ros/camera_info

And make a file called `head_camera.yaml` and copy the text below:

    image_width: 64
    image_height: 480
    camera_name: head_camera
    camera_matrix:
      rows: 3
      cols: 3
      data: [1965.198759922371, 0, 162.1142583645783, 0, 1968.887033831632, 393.7339989757747, 0, 0, 1]
    distortion_model: plumb_bob
    distortion_coefficients:
      rows: 1
      cols: 5
      data: [0.05352954540529466, 0.3532778332300517, 0.02565579611337958, -0.04127488682464868, 0]
    rectification_matrix:
      rows: 3
      cols: 3
      data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    projection_matrix:
      rows: 3
      cols: 4
      data: [1940.791625976562, 0, 156.3951561000204, 0, 0, 1967.11376953125, 395.9012302382544, 0, 0, 0, 1, 0]

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

OR

    roslaunch fetch_following usb_cam_stream_publisher.launch 


Follow the aruco instructions here:
---
http://ros-developer.com/2017/04/23/aruco-ros/

Or follow these steps:

Step 1. Open a new terminal and source it!

    source devel/setup.bash

Step 2. launch the aruco:

    roslaunch fetch_following aruco_marker_finder.launch markerId:=701 markerSize:=0.1

Change `markerId:=701 markerSize:=0.1` to match the specs of your artag or change this directly in the launch. For this project, it has already been changed.

Run 

    rosrun rqt_gui rqt_gui

to view the output. If you see a blank screen, do not fear, help is here:

1. Plugins > Visualization > Image View
2. Then select your topic

The topic where we will see the pose is in: 

    /aruco_single/result

You can echo /aruco_single/pose to get the pose of the tag

    rostopic echo /aruco_single/pose

To see what its subscribed to:

    rostopic info /aruco_single/pose

**Extra:**

To check out where the aruco-ros installation location:

    roscd aruco_ros

To generate the tag, go to: http://chev.me/arucogen/

***This is important.*** **Aruco has a library of saved IDs. It will need an ID from this library to know what ID it is.**

To view your tf, use:

    rosrun tf view_frames

**Note:** the pose will have to be actively running and picking up messages in order for you too see the tf tree.

# To Run the Pose detector

Terminal 1:

    roslaunch fetch_following aruco_marker_finder.launch 

# To get the rosbag

rosbag record <topics to record> <another topic> -O <name of the file>

    rosbag record /usb_cam/image_raw /usb_cam/camera_info -O following_3

# To run the simulation

Follow the first few instructions from this site to install the fetch simulation:

https://docs.fetchrobotics.com/gazebo.html

Once installed, enter this in a new terminal:

    roslaunch fetch_gazebo simulation.launch

# To run the fetch motion control

```bash
rosrun fetch_following motion.py
```

to make it rosrun-able(executable)

    chomd +x motion.py

then you can `rosrun fetch_following motion.py`

To control the fetch with keyboard commands
---

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

**Some extra useful commands:**

    rostopic echo /odom

    rosmsg show Twist


# How to be a Parasite (connecting to a host and sharing a roscore)

    cd /etc
    subl hosts

Add the host's ip and computer's name. 

eg:
    
    <192.3.5.234> <name of the host computer>
 

If you don't know your ip address, use

    ifconfig

and find the ip besides 'inet addr'

**Extra:** you and your host must share the same network (wifi or ethernet connection)
