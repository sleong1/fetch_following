# how to make it move
import rospy
import numpy as np
    
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
        self.last_received_range_time = None

    def laser_cb(self, msg):
        # For implementing safety
        self.laser_readings = msg
        self.last_received_range_time = rospy.Time.now()
        x, turn = self.avoid_obstacle(msg)
        self.publish_cmd_vel(None, x, 0, turn)

    def publish_cmd_vel(self, msg=None,x=0,y=0,turn=0):
        ''' x is the velocity in this direction
            y is the velocity in thisi direction
            turn is the angular velocity in this rotation'''
        if not msg:
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.angular.z = turn
        self.cmd_vel_pub.publish(msg)

# range_min: 0.0500000007451
# range_max: 25.0
# angle_min: -1.91986000538
# angle_max: 1.91986000538
# angle_increment: 0.00580895598978
    def avoid_obstacle(self, msg, threshold=2):

        ''' threshold is the distance you want the robot to be away 
            from anything. In metres(m).
                  ^theta
              w = -------
                   ^time

            where ^ time is how long we want the robot to take to make that turn

            '''
        # scanning through all the angles.

        distance = msg.ranges
        # print('the size of the ranges array: ' + str(len(distance)))
        max_range = msg.range_max
        range_angle_index = []
        range_angle = []

        for each_laser_distance in distance:
            if each_laser_distance > threshold:
                range_angle_index.append(distance.index(\
                                         each_laser_distance))
        
        # have to sort to get the meaningful indexs.
        grouped_ranges = self.grouping(range_angle_index)
        # print('\n grouped_ranges: ' + str(grouped_ranges))
        max_group_index = 0
        max_group = 0
        for each_group in grouped_ranges:
            # print('\n each_group: ' + str(each_group))
            group_size = len(each_group)
            # print('\n group_size: ' + str(group_size))
            if group_size > max_group:
                max_group = group_size
                max_group_index = grouped_ranges.index(each_group)
        print('\n max_group_index: ' + str(max_group_index))
        # now we have the index of the group that will have the 
        # largest room for the robot to travel

        # get the average of the range_angle
        angle_mean = round(np.mean(grouped_ranges[max_group_index]))
        print('\n angle_mean: ' + str(angle_mean))
        print('\n msg.ranges[331]: ' + str(msg.ranges[331]))

        adjust_angle = float(angle_mean * msg.angle_increment)
        delta_time = 1
        deg_90_in_rad = 1.5708
        if adjust_angle > deg_90_in_rad:
            # ensures that it will turn clockwise
            delta_theta = -(adjust_angle - deg_90_in_rad)
        elif adjust_angle < deg_90_in_rad:
            delta_theta = deg_90_in_rad - adjust_angle
        adjust_rotation = delta_theta / delta_time

        time_now = rospy.Time.now()
        # convert angle to angular velocity
        # angular_velocity = (msg.ranges[331] - angle_mean)/ (time_now - self.last_received_range_time) 
        # print('\n angular_velocity: ' + str(angular_velocity))

        #populate the dist within certain limit
        dist = []
        y_dist_left = distance[30]
        y_dist_right = distance[632]
        print('\n y_dist_right distance[632]: ' + str(y_dist_right))
        print('\n y_dist_left distance[30]: ' + str(y_dist_left))

        hallway_threshold = 1
        if y_dist_left < hallway_threshold:
             while y_dist_left < hallway_threshold:
                print('Re centring')
                x = 0.5
                turn = -0.5
                # turn = angular_velocity
                print float(x), float(turn)
                return float(x), float(turn)
        if y_dist_right < hallway_threshold:
             while y_dist_left < hallway_threshold:
                print('Re centring')
                x = 0.5
                turn = 0.5
                # turn = angular_velocity
                print float(x), float(turn)
                return float(x), float(turn)
        for i in range(300,360):
            dist = distance[i]
        # dist = distance[331]
        # for front_scanner in dist:
            if dist < threshold:
                while dist < threshold:
                    # we want the robot to stop moving
                    print('Turn TURN TURN!')
                    x = 0.5
                    turn = adjust_rotation

                    if adjust_rotation < 0:
                        print('Turn TURN TURN right!')
                    else:
                        print('Turn TURN TURN left!')

                    # turn = angular_velocity
                    print float(x), float(turn)
                    return float(x), float(turn)
        x = 1
        turn = 0
        print float(x), float(turn)
        return float(x), float(turn)

    # if avoid_obstacle is done-> go back to following person

    def grouping(self, range_list): 
        ''' Sorting the list of ranges into smaller groups
            where each group have increasing numbers without
            any gaps.
            eg. [[1,2,3], [6,7,8]]'''
        range_group = [[range_list[0]]]      
        for i in range(1, len(range_list)): 
            if range_list[i-1]+1 == range_list[i]: 
                range_group[-1].append(range_list[i]) 
            else: 
                range_group.append([range_list[i]]) 
        return range_group 
       
if __name__ == '__main__':
    
    c = Controller()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # c.publish_cmd_vel(x=0,y=0,turn=1) 
        rate.sleep()
        # c.get_laser_scan_messages()
        rospy.spin()


    # c.publish_cmd_vel(msg=None,x=0,y=0,turn=1.6)


'''
max negative angle is -109.9998755645104 degrees
and max positive angle is 109.9998755645104 degrees
'''

