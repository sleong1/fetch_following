# how to make it move
import rospy
    
from geometry_msgs.msg import PoseStamped, Twist

class Controller(object):
    def __init__(self):
        rospy.init_node('move')
        self.cmd_vel_pub = rospy.Publisher('/base_controller/command',
                                           Twist, queue_size=1)


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

