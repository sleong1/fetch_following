'''
Activating the camera in 1 terminal:
roslaunch usb_cam usb_cam-test.launch
in another terminal, run this script
'''

import numpy as np
import time
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

class QRReader(object):

    def __init__(self):
        # if not rospy.is_initialized():
        # rospy.init_node('getting_image')
        # Instantiate CvBridge
        self.bridge = CvBridge()

    def callback_image(self, msg):
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite('data/camera_image.jpeg', cv2_img)

    def main(self):
        rospy.init_node('image_listener')
        # Define your image topic
        image_topic = "/usb_cam/image_raw"
        # Set up your subscriber and define its callback
        rospy.Subscriber(image_topic, Image,
                         self.callback_image,
                         queue_size=1)
        # Spin until ctrl + c
        rospy.spin()

if __name__ == '__main__':
    qr = QRReader()
    qr.main()














