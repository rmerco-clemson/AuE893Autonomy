#!/usr/bin/env python

"""

"""
import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class turtlebot_laser:

    def __init__(self):
        self.velocity_publisher = rospy.Publisher(
            'turtle1/cmd_vel', Twist, queue_size=5)
        self.odom_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.odeometry_callback, queue_size=5)
        self.laser_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.laser_callback, queue_size=5)
        self.min_distance = 10
        self.Flag_duck = False
        self.Flag_turn_left = True
        self.Flag_arrived_pos = False
        self.alarm_level = "Green"
        self.obs_place = "N/A"
        self.turtlebot_odom = Odometry()
        self.turtlebot_laser = LaserScan()

    def quanternion_to_yaw(self, odom_msg):
        ''' Convert quanternion to yaw in radians '''
        qt_x = odom_msg.pose.pose.orientation.x
        qt_y = odom_msg.pose.pose.orientation.y
        qt_z = odom_msg.pose.pose.orientation.z
        qt_w = odom_msg.pose.pose.orientation.w

        y = 2.0 * (qt_w * qt_z + qt_x * qt_y)
        x = 1.0 - 2.0 * (qt_y * qt_y + qt_z * qt_z)

        return math.atan2(y, x)

    def odeometry_callback(self, odom_msg):
        ''' call back function of position of turtle'''
        self.turtlebot_odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        self.turtlebot_odom.pose.pose.position.y = odom_msg.pose.pose.position.y

        self.turtlebot_odom.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        self.turtlebot_odom.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        self.turtlebot_odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        self.turtlebot_odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        print "x=%.3f, y=%.3f, yaw=%.3f" % (
            self.turtlebot_odom.pose.pose.position.x,
            self.turtlebot_odom.pose.pose.position.y,
            math.degrees(self.quanternion_to_yaw(self.turtlebot_odom)))

    def obstacle_detection(self):
        ''' Deal with obstacle with laser scan '''

        entries = len(self.turtlebot_laser.ranges)
        self.min_distance = self.turtlebot_laser.range_max
        min_point = -1
        # Get nearest point
        for entry in range(0, entries):
            if 0.4 < self.turtlebot_laser.ranges[entry] < 9.5:
                if self.turtlebot_laser.ranges[entry] < self.min_distance:
                    self.min_distance = self.turtlebot_laser.ranges[entry]
                    min_point = entry
        # Set safe level
        if self.min_distance < 0.6:
            self.alarm_level = "Red"
            self.Flag_duck = True
        elif self.min_distance < 2.0:
            self.alarm_level = "Yellow"
            self.Flag_duck = False
        else:
            self.alarm_level = "Green"
            self.Flag_duck = False
        # Set obstacle direction
        if min_point == -1:
            self.obs_place = "N/A"
        elif 0 <= min_point < (entries // 2):
            self.obs_place = "Right"
            self.Flag_turn_left = True
        else:
            self.obs_place = "Left"
            self.Flag_turn_left = False

        print "Arlarm Level: %s \t Obstacle in the %s \t Distance: %.3f" % (
            self.alarm_level, self.obs_place, self.min_distance)

    def laser_callback(self, laserscan):
        '''Callback function for laser scan '''
        self.turtlebot_laser.ranges = laserscan.ranges
        self.turtlebot_laser.angle_min = laserscan.angle_min
        self.turtlebot_laser.angle_max = laserscan.angle_max
        self.turtlebot_laser.range_min = laserscan.range_min
        self.turtlebot_laser.range_max = laserscan.range_max

        self.obstacle_detection()

    def constrain(self, val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))

    def get_distance(self, x1, y1, x2, y2):
        ''' Calculate distance between (x1, y1) and (x2, y2)'''
        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def set_orientation(self, desired_yaw=0, tol=5e-4):
        ''' Turn to a certain orientation through PID
        input: desired yaw in radians.'''
        # avoid angularity
        if desired_yaw > math.pi:
            desired_yaw -= (2 * math.pi)

        # Define vel_msg
        vel_msg = Twist()
        # loop rate
        control_frequency = 30
        rate = rospy.Rate(control_frequency)

        error = 0.0
        last_error = 0.0
        d_error = 0.0
        i_error = 0.0

        while not rospy.is_shutdown():
            # define Kp and Ki
            Kp = 5.0
            Ki = 0.003
            Kd = 2.0
            # Calculate error
            error = desired_yaw - self.quanternion_to_yaw(self.turtlebot_odom)
            if error > math.pi:
                error -= (2 * math.pi)
            elif error < -math.pi:
                error += (2 * math.pi)

            d_error = (error - last_error) * control_frequency

            if math.fabs(error) < tol:
                break
            # generate vel cmd
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.constrain(
                Kp * error + Ki * i_error + Kd * d_error,
                -math.radians(10), math.radians(10))

            self.velocity_publisher.publish(vel_msg)

            if i_error < 1.5:
                i_error += error / control_frequency
            else:
                i_error = 0
            last_error = error

            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def go_to_pos(self, pos_x=0.0, pos_y=0.0, tol_distance=0.01):
        ''' Go to position (x,y) with PID'''

        # Reset Flag arrived pos
        self.Flag_arrived_pos = False

        # # get current position
        # current_x = self.turtlebot_odom.pose.pose.position.x
        # current_y = self.turtlebot_odom.pose.pose.position.y
        # # Get current distance to the goal
        # current_distance = self.get_distance(
        #     current_x, current_y, pos_x, pos_y)

        # # Calculate angle
        # target_yaw = math.atan2((pos_y - current_y), (pos_x - current_x))
        # # Go to target orientation
        # self.set_orientation(target_yaw)
        # # Stop for 1 second
        # time.sleep(1)

        # Move straight to desired position.

        # Set limit of x and y
        x_min = -4.5
        x_max = 4.5
        y_min = -4.5
        y_max = 4.5

        destination = Odometry()
        destination.pose.pose.position.x = self.constrain(pos_x, x_min, x_max)
        destination.pose.pose.position.y = self.constrain(pos_y, y_min, y_max)
        # Define vel_msg
        vel_msg = Twist()
        # loop rate
        control_frequency = 30
        rate = rospy.Rate(control_frequency)

        # Init parameters of distance pid
        error_distance = 0.0
        last_error_distance = 0.0
        d_error_distance = 0.0
        i_error_distance = 0.0

        # Init parameters of orientation pid
        error_orientation = 0.0
        last_error_orientation = 0.0
        d_error_orientation = 0.0
        i_error_orientation = 0.0

        while self.Flag_duck is False and self.Flag_arrived_pos is False and \
                not rospy.is_shutdown():
            last_distance = current_distance

            # define Kp, Ki, Kd
            Kp_distance = 0.8
            Ki_distance = 0.01
            Kd_distance = 1.8
            Kp_orientation = 8.0
            Ki_orientation = 0.05
            Kd_orientation = 8.0

            # Calculate error
            error_distance = self.get_distance(
                self.turtlebot_odom.pose.pose.position.x,
                self.turtlebot_odom.pose.pose.position.y,
                pos_x, pos_y)

            error_orientation = target_yaw - \
                self.quanternion_to_yaw(self.turtlebot_odom)
            # avoid angularity
            if error_orientation > math.pi:
                error_orientation -= (2 * math.pi)
            elif error_orientation < -math.pi:
                error_orientation += (2 * math.pi)

            d_error_distance = (
                error_distance - last_error_distance) * control_frequency
            d_error_orientation = (error_orientation -
                                   last_error_orientation) * control_frequency

            # Check if arrived or not
            if error_distance < tol_distance:
                self.Flag_arrived_pos = True
                break

            # Generate vel cmd
            vel_msg.linear.x = self.constrain(
                Kp_distance * error_distance +
                Ki_distance * i_error_distance +
                Kd_distance * d_error_distance, -0.15, 0.15)

            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            vel_msg.angular.z = self.constrain(
                Kp_orientation * error_orientation
                + Ki_orientation * i_error_orientation +
                Kd_orientation * d_error_orientation,
                -math.radians(10),
                math.radians(10))

            self.velocity_publisher.publish(vel_msg)

            # Verify distance
            current_x = self.turtlebot_odom.pose.pose.position.x
            current_y = self.turtlebot_odom.pose.pose.position.y
            current_distance = self.get_distance(current_x, current_y, pos_x, pos_y)

            if current_distance > last_distance + 0.05:
                self.Flag_arrived_pos = False
                break

            # Update parameters
            if i_error_distance < 1:
                i_error_distance += error_distance / control_frequency
            else:
                i_error_distance = 0

            last_error_distance = error_distance

            if i_error_orientation < 1.5:
                i_error_orientation += error_orientation / control_frequency
            else:
                i_error_orientation = 0

            last_error_orientation = error_orientation

            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def duck(self, turn_left=True, angle=45, distance=0.2):
        ''' Make a duck to avoid the obstacle '''
        current_x = self.turtlebot_odom.pose.pose.position.x
        current_y = self.turtlebot_odom.pose.pose.position.y
        current_yaw = self.quanternion_to_yaw(self.turtlebot_odom)

        if turn_left:
            desired_yaw = current_yaw + math.radians(angle)
        else:
            desired_yaw = current_yaw - math.radians(angle)

        desired_x = current_x + distance * math.cos(desired_yaw)
        desired_y = current_y + distance * math.sin(desired_yaw)

        self.go_to_pos(desired_x, desired_y)

    def move_to_goal(self, pos_x=0.0, pos_y=0.0, tol_distance=0.05):
        ''' Move to a position with obstacle avoidance'''
        self.Flag_arrived_pos = False

        while not rospy.is_shutdown():
            if self.Flag_duck:
                self.duck(self.Flag_turn_left)
            else:
                self.go_to_pos(pos_x, pos_y, tol_distance)
                if self.Flag_arrived_pos:
                    break

        # Define vel_msg
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)



def main():
    '''main function for the task '''

    print "Init ROS node ..."

    turtlebot = turtlebot_laser()

    rospy.init_node("turtlebot_laser")

    print "Move to (3.0, 0.0)"
    turtlebot.move_to_goal(3.0, 0.0)
    print "Arrived."

    print "Move to (-2.4, -1.0)"
    turtlebot.move_to_goal(-2.4, -1.0)
    print "Arrived."

    print "Done."

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass