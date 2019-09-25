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
        x, turn = self.avoid_wall(msg)
        # print('\nranges:')
        # print msg.ranges
        self.publish_cmd_vel(None, x, 0, turn)

    def publish_cmd_vel(self, msg=None,x=0,y=0,turn=0):
        '''
            x is the velocity in this direction
            y is the velocity in thisi direction
            turn is the angular velocity in this rotation

        '''
        # publish cmd_vel messages
        if not msg:
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.angular.z = turn
        self.cmd_vel_pub.publish(msg)

    def avoid_wall(self, msg, threshold=1):
        '''
            threshold is the distance you want the robot to be away 
            from anything. In metres(m).
        '''
        # this is the value that is straight ahead of the robot
        distance = msg.ranges[360] 
        print(distance)
        if distance < threshold:
            while distance < threshold:
                # we want the robot to stop moving
                print('stop that robot!')
                x = 0
                turn = 1
                print x, turn
                return x, turn
        x = 1
        turn = 0
        print x, turn
        return x, turn



if __name__ == '__main__':
    
    c = Controller()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # c.publish_cmd_vel(x=1,y=1,turn=0) 
        # rate.sleep()
        # c.get_laser_scan_messages()
        rospy.spin()
'''
max negative angle is -109.9998755645104 degrees
and max positive angle is 109.9998755645104 degrees
'''

