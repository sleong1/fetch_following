# how to make it move
import rospy
    
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan


class Controller(object):
    def __init__(self):
        rospy.init_node('move')
        self.laser_readings = None
        
        self.cmd_vel_pub = rospy.Publisher('/base_controller/command',
                                           Twist, queue_size=1)
        
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan,
                                          self.laser_cb, queue_size=1)

    def laser_cb(self, msg):
        # For implementing safety
        self.laser_readings = msg


    def publish_cmd_vel(self, msg=None,x=0,y=0,turn=0):
        # publish cmd_vel messages
        if not msg:
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.angular.z = turn
        self.cmd_vel_pub.publish(msg)


if __name__ == '__main__':
    
    c = Controller()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        c.publish_cmd_vel(x=-1,y=1,turn=0) 
        rate.sleep()


'''
max negative angle is -109.9998755645104 degrees
and max positive angle is 109.9998755645104 degrees
'''

