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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_cb, queue_size=1)
        self.human_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.human_detection, queue_size=1)
        self.last_received_pose_time = None
        self.aruco_pose = None
        self.max_speed = 1.0 #m/s
        self.max_rads = 1.0 # rad/s
        self.vel = 0.0
        self.rot = 0.0
        while not self.aruco_pose:
            rospy.sleep(0.1)
        print("Finished initialising Motion module")

    def laser_cb(self, msg):
        # For implementing safety
        self.laser_readings = msg

    def human_detection(self, msg):
        self.aruco_pose = msg.pose
        self.last_received_pose_time = rospy.Time.now()
        self.send_speed_command(new=True)

    def send_speed_command(self, new=False):
        if new is True:
            self.vel, self.rot = self.convert_to_vel(self.aruco_pose)
        self.publish_cmd_vel(self.vel, self.rot)

    def get_distance(self, x1, y1, z1=0, x2=0, y2=0, z2=0):
        return sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

    def convert_to_vel(self, pose, threshold=0.8):
        # Get the relative distance from the robot to the human
        # Use it for speed scaling

        distance = self.get_distance(pose.position.x, pose.position.y, pose.position.z)
        # Multiplied by 10 because it is in wrong scale.
        print("distance is: " + str(distance))

        threshold_dist=2.0
        scale = distance/(1.25*threshold_dist)
        
        x_offset = pose.position.x
        # print("x_offset is:" + str(x_offset))
        # print(x_offset/distance)
        # print pose.position.x
        rot_scale = 5*asin(x_offset/distance)/(pi/4)
        # print("x position is: ", pose.position.x)
        # print("scale is: ", rot_scale)

        if abs(rot_scale) > 1:
            rot_scale = rot_scale/abs(rot_scale)
        # Simulation is opposite to camera frame ;/
        rotation = -rot_scale * self.max_rads
        # print(rotation*(180/pi))

        backwards_threshold = 0.4

        if distance < threshold:
            if distance >= backwards_threshold:
                print("too close to human, not moving linearly")
                return 0.0, rotation
            else:
                print("too close to human, moving backwards")
                return -0.2, rotation
        # 1.25 is so when at threshold distance, will travel
        # at 0.8 m/s ~ approximately human walking speed
        if scale > 1:
            scale = 1.0
        velocity = scale * self.max_speed

        print("robot is moving", velocity, rotation)
        return velocity, rotation

    def publish_cmd_vel(self, velocity=0, rotation=0):
        # publish cmd_vel messages
        msg = Twist()
        msg.linear.x = velocity
        msg.angular.z = rotation
        self.cmd_vel_pub.publish(msg)

    def main(self):
        self.last_received_pose_time = rospy.Time.now()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():    
            self.send_speed_command()
            if (rospy.Time.now() - self.last_received_pose_time).to_sec > 5:
                self.vel = 0
                self.rot = 0
            r.sleep()
            # Will wait for messages forever
            # rospy.spin()
            # Will publish empty cmd_vel messages every second forever
            # self.publish_cmd_vel(Twist())
            # rospy.sleep(1.0)


if __name__ == "__main__":
    motion = Motion()
    motion.main()
