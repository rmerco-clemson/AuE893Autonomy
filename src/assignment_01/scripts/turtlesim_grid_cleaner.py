#!/usr/bin/env python

"""

"""
import math
import random
import time
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def main():
    ''' '''

    turtlesim_pose = Pose()
    def pose_callback(pos_msg):
        ''' call back function of position of turtle'''
        turtlesim_pose.x = pos_msg.x
        turtlesim_pose.y = pos_msg.y
        turtlesim_pose.theta = pos_msg.theta
        print"px = %.2f, py = %.2f, theta = %.2f" %(
            turtlesim_pose.x,
            turtlesim_pose.y,
            math.degrees(turtlesim_pose.theta))

    # publisher for cmd_vel to turtlesim
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber(
        "/turtle1/pose", Pose, pose_callback, queue_size=5)
    rospy.init_node('turtlesim_cleaner', anonymous=True)

    def move(speed=0, distance=0, is_forward=True):
        '''let the turtle move straight in turtlesim '''

        # define velocity command
        vel_msg = Twist()
        if is_forward == True:
            vel_msg.linear.x = math.fabs(speed)
        else:
            vel_msg.linear.x = -math.fabs(speed)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # loop rate
        loop_rate = rospy.Rate(100)  # 10hz
        time_start = rospy.get_time()
        moved_distance = 0
        while not rospy.is_shutdown():
            time_now = rospy.get_time()
            moved_distance = math.fabs(speed) * (time_now - time_start)
            if moved_distance >= math.fabs(distance):
                break
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

        # set speed back to zero
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)

    def rotate(angular_speed=0, theta=0, is_clockwise=True):
        '''Let the turtle rotate in turtlesim'''

        # Define vel_msg
        vel_msg = Twist()
        # Set linear speed to zero
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        # Set angular speed in x, y axes to zero
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        if is_clockwise == True:
            vel_msg.angular.z = -math.fabs(math.radians(angular_speed))
        else:
            vel_msg.angular.z = math.fabs(math.radians(angular_speed))

        # loop rate
        loop_rate = rospy.Rate(100)  # 10hz
        time_start = rospy.get_time()
        rotated_angle = 0
        while not rospy.is_shutdown():
            time_now = rospy.get_time()
            rotated_angle = angular_speed * (time_now - time_start)
            if rotated_angle >= math.fabs(theta):
                break
            velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

        # Set linear speed to zero
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        # Set angular speed in x, y axes to zero
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0


    def go_to_pos(pos_x=5.0, pos_y=5.0, tol=1e-3):
        ''' Let the turtle go to certain x,y in turtlesim'''
        # Set limit of x and y
        x_min = 0
        x_max = 11.0
        y_min = 0
        y_max = 11.0

        def constrain(val, min_val, max_val):
            ''' Constrain val between min and max'''
            return min(max_val, max(min_val, val))

        def get_distance(x1, y1, x2, y2):
            ''' Calculate distance between (x1, y1) and (x2, y2)'''
            return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

        destination = Pose()
        destination.x = constrain(pos_x, x_min, x_max)
        destination.y = constrain(pos_y, y_min, y_max)
        # Define vel_msg
        vel_msg = Twist()
        # loop rate
        control_frequency = 100
        rate = rospy.Rate(control_frequency)
        error = 0.0
        last_error = 0.0
        d_error = 0.0
        i_error = 0.0
        while not rospy.is_shutdown():
            # define Kp and Ki
            Kp = 1.0
            Ki = 0.001
            Kd = 0.01
            # Calculate error
            error = get_distance(
                turtlesim_pose.x, turtlesim_pose.y,
                pos_x, pos_y)
            if error < tol:
                break

            # generate vel cmd
            vel_msg.linear.x = Kp * error + Ki * i_error + Kd * d_error
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 12 * (math.atan2(
                pos_y - turtlesim_pose.y,
                pos_x - turtlesim_pose.x) - turtlesim_pose.theta)

            velocity_publisher.publish(vel_msg)

            if i_error < 1:
                i_error += error / control_frequency
            else:
                i_error = 0
            d_error = (error - last_error) / control_frequency
            last_error = error

            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

    def set_orientation(desired_theta=0):
        ''' turn turtle to a certain orientation '''
        turning_angle = desired_theta - math.degrees(turtlesim_pose.theta)
        if turning_angle < 0:
            turning_clockwise = True
        else:
            turning_clockwise = False
        rotate(
            angular_speed=30,
            theta=turning_angle,
            is_clockwise=turning_clockwise)

    def grid_clean_10_by_10():
        '''  '''

        print"Start the task: grid clean the room 10 by 10."
        go_to_pos(0.5, 0.5)

        for i in range(5):
            set_orientation(0)
            move(2.5, 10, True)
            set_orientation(90)
            move(2.5, 1, True)
            set_orientation(180)
            move(2.5, 10, True)
            set_orientation(90)
            move(2.5, 1, True)

        set_orientation(0)
        move(2.5, 10, True)

        print "Task is completed!"

    def spiral_clean_10_by_10():
        ''' '''

        print"Start the task: sparial clean the room 10 by 10."

        go_to_pos(5.54444445, 5.544445)
        vel_msg = Twist()
        loop_rate = rospy.Rate(1)
        rk = 0.5
        constant_speed = 4
        while not rospy.is_shutdown():
            if turtlesim_pose.x >= 10.5 or turtlesim_pose.y >= 10.5:
                break
            vel_msg.linear.x = rk
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = constant_speed

            velocity_publisher.publish(vel_msg)
            rk += 0.8
            loop_rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)







    #####################################################
    # Test code
    #####################################################

    # go_to_pos(6, 6)
    # move(1, 2, True)
    # rotate(30, 60, True)
    # move(1, 2, True)
    # rotate(30, 60, True)
    # move(1, 2, True)
    # rotate(30, 60, True)
    # move(1, 2, True)
    # rotate(30, 60, True)
    # move(1, 2, True)
    # rotate(30, 60, True)
    # move(1, 2, True)
    # rotate(30, 60, True)


    # while not rospy.is_shutdown():

    #     x = 11 * random.random()
    #     y = 11 * random.random()

    #     print "x = %2f, y = %2.f" %(x,y)

    #     go_to_pos(x, y)

    # go_to_pos(1, 1)
    # go_to_pos(6, 6)
    # go_to_pos(10, 1)
    # go_to_pos(6, 6)
    # go_to_pos(1, 11)
    # go_to_pos(6, 8)
    # go_to_pos(7, 1)
    # go_to_pos(9, 5)

    # set_orientation(0)
    # time.sleep(5)
    # set_orientation(180)
    # time.sleep(5)
    # set_orientation(0)
    # time.sleep(5)
    # set_orientation(180)

    grid_clean_10_by_10()
    # spiral_clean_10_by_10()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
