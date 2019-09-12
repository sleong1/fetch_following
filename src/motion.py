#!/usr/bin/env python

from math import sqrt, asin, pi

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseActionGoal

class Motion(object):
    def __init__(self):
        rospy.init_node('fetch_motion')
        self.laser_readings = None
        self.cmd_vel_pub = rospy.Publisher('/base_controller/command', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_cb, queue_size=1)
        self.human_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.human_detection, queue_size=1)
        print("Finished initialising Motion module")

    def laser_cb(self, msg):
        # For implementing safety
        self.laser_readings = msg

    def human_detection(self, msg):
        vel, rot = self.convert_to_vel(msg.pose)
        self.publish_cmd_vel(vel, rot)

    def get_distance(self, x1, y1, x2=0, y2=0, z1=0, z2=0):
        return sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

    def convert_to_vel(self, pose, threshold_dist=2.0, human_dist=0.8):
        # Get the relative distance from the robot to the human
        # Use it for speed scaling
        max_speed = 1.0 #m/s also rad/s
        distance = self.get_distance(pose.position.x, pose.position.y, pose.position.z)
        # print("distance is: " + str(distance))
        # print(distance)
        scale = distance/(1.25 * threshold_dist)
        
        # For some reason, the middle is not in the centre
        x_offset = pose.position.x + 0.2

        # print pose.position.x
        rot_scale = 5*(asin(x_offset/distance)/(pi/4))
        # print("x position is: ", pose.position.x)
        # print("scale is: ", rot_scale)

        if rot_scale > 1:
            rot_scale = 1
        rotation =  rot_scale * max_speed
        # print(rotation*(180/pi))


        if distance < human_dist:
            print("not moving linearly, too close to human")
            return 0.0, rotation
        # 1.25 is so when at threshold distance, will travel
        # at 0.8 m/s ~ approximately human walking speed
        if scale > 1:
            scale = 1.0
        velocity = scale * max_speed

        # print(velocity)
        return velocity, rotation

    def publish_cmd_vel(self, velocity=0, rotation=0):
        # publish cmd_vel messages
        msg = Twist()
        msg.linear.x = velocity
        msg.angular.z = rotation
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
