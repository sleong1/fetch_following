#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

class Dodge(object):
    def __init__(self):
        self.fetch_base_width = 0.508 #(m)
        

        rospy.init_node('dodge')
        self.laser_readings = None
        self.last_received_range_time = None
        # self.cmd_vel_pub = rospy.Publisher('/base_controller/command', Twist, queue_size=1)
        self.cmd_rot_pub = rospy.Publisher('/rot_cmd', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_cb, queue_size=1)

    def laser_cb(self, msg):
        # For implementing safety
        self.laser_readings = msg
        self.publish_cmd_vel()

    def publish_cmd_vel(self, msg=None,x=0,y=1,turn=0):
        ''' x is the velocity in this direction
            turn is the angular velocity in this rotation'''
        if not msg:
            msg = Twist()
            msg.linear.x = x
            msg.angular.z = turn
        self.cmd_vel_pub.publish(msg)

    def find_boundary(self, msg, side_threshold, front_threshold):
        angle_increment = msg.angle_increment
        centre_index = len(msg.ranges)/2
        
        theta = (math.atan2(side_threshold,front_threshold))/angle_increment
        min_index = centre_index - theta
        max_index = centre_index + theta
        
        return int(min_index), int(max_index)

    def dodge(self, msg, side_threshold=1, front_threshold=2):
        min_angle, max_angle = self.find_boundary(msg, side_threshold, front_threshold)
        # for i in range(min_angle, max_angle + 1):
        #     distance = msg.ranges[i]
        #     if distance < front_threshold:
        #         y_vel = msg.range[331]/



    def main(self):
        pass


       
if __name__ == '__main__':
    d = Dodge()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # d.publish_cmd_vel(x=1,y=0,turn=0) 
        rate.sleep()
        rospy.spin()



