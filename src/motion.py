#!/usr/bin/env python

from math import sqrt

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformation import EulerFromQuarternion
from move_base_msgs.msg import MoveBaseActionGoal

class Motion(object):
    def __init__(self):
        if not rospy.is_initialized():
            rospy.init_node('fetch_motion')
        self.laser_readings = None
        self.cmd_vel_pub = rospy.Publisher('/base_controller/command', Twist(), queue_size=1)
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_cb, queue_size=1)
        self.human_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.human_detection, queue_size=1)

    def laser_cb(self, msg):
        # For implementing safety
        self.laser_readings = msg

    def human_detection(self, msg):
        vel = self.convert_to_vel(msg.pose)
        self.publish_cmd_vel(vel)


    def get_distance(self, x1, y1, x2=0, y2=0):
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def convert_to_vel(self, pose):
        # Get the relative distance from the robot to the human
        # Use it for speed scaling
        distance = self.get_distance(pose.position.x, pose.position.y)
        
        pass

    def publish_cmd_vel(self, msg=None):
        # publish cmd_vel messages
        if not msg:
            msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def main(self):
        while not rospy.is_shutdown():
            # Will wait for messages forever
            rospy.spin()
            # Will publish empty cmd_vel messages every second forever
            # self.publish_cmd_vel(Twist())
            # rospy.sleep(1.0)


if __name__ == "__main__":
    motion = Motion()
    motion.main()
