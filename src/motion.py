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
        self.human_sub = rospy.Subscriber('/human_position', PoseStamped, self.human_detection, queue_size=1)
        self.human_position = None

    def laser_cb(self, msg):
        # For implementing safety
        self.laser_readings = msg

    def human_detection(self, msg):
        vel = self.convert_to_vel(msg)
        self.publish_cmd_vel(vel)


    def convert_to_vel(self, pose):
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
