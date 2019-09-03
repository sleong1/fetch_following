#!/usr/bin/env python

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
        self.cmd_vel_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal(), queue_size=1)
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_cb, queue_size=1)

    def laser_cb(self, msg):
        self.laser_readings = msg

    def publish_cmd_vel(self, msg=None):
        # publish cmd_vel messages
        if not msg:
            msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def do_something(self):
        self.publish_cmd_vel(Twist())

    def main(self):
        while not rospy.is_shutdown():
            # Will publish empty cmd_vel messages every second forever
            self.do_something()
            rospy.sleep(1.0)


if __name__ == "__main__":
    motion = Motion()
    motion.main()
