#!/usr/bin/env python

"""

"""
import math
import random
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def main():
    ''' '''

    turtlebot_odom = Odometry()

    def odeometry_callback(odom_msg):
        ''' call back function of position of turtle'''
        turtlebot_odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        turtlebot_odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        turtlebot_odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z

        rospy.loginfo("px=%.2f, py=%.2f, theta=%.1f ",
                      turtlebot_odom.pose.pose.position.x,
                      turtlebot_odom.pose.pose.position.y,
        math.degrees(2 * math.asin(turtlebot_odom.pose.pose.orientation.z)))



    # publisher for cmd_vel to turtlesim
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber(
        "/odom", Odometry, odeometry_callback, queue_size=5)
    rospy.init_node('turtlesim_cleaner', anonymous=True)

    def constrain(val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))

    def move(speed=0, distance=0, is_forward=True):
        '''let the turtle move straight in turtlesim '''

        # define velocity command
        vel_msg = Twist()
        if is_forward == True:
            vel_msg.linear.x = constrain(speed, 0, 0.4)
        else:
            vel_msg.linear.x = - constrain(speed, 0, 0.4)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # loop rate
        loop_rate = rospy.Rate(30)  # 10hz
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
            vel_msg.angular.z = -math.radians(constrain(angular_speed, 0, 30))
        else:
            vel_msg.angular.z = math.radians(constrain(angular_speed, 0, 30))

        # loop rate
        loop_rate = rospy.Rate(30)  # 10hz
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
        x_min = -4.5
        x_max = 4.5
        y_min = -4.5
        y_max = 4.5

        # def constrain(val, min_val, max_val):
        #     ''' Constrain val between min and max'''
        #     return min(max_val, max(min_val, val))

        def get_distance(x1, y1, x2, y2):
            ''' Calculate distance between (x1, y1) and (x2, y2)'''
            return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

        destination = Odometry()
        destination.pose.pose.position.x = constrain(pos_x, x_min, x_max)
        destination.pose.pose.position.y = constrain(pos_y, y_min, y_max)
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
            Kp = 1.0
            Ki = 0.001
            Kd = 0.01
            # Calculate error
            error = get_distance(
                turtlebot_odom.pose.pose.position.x,
                turtlebot_odom.pose.pose.position.y,
                pos_x, pos_y)
            if error < tol:
                break

            # generate vel cmd
            vel_msg.linear.x = constrain(
                Kp * error + Ki * i_error + Kd * d_error, -0.4, 0.4)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = constrain(
                4 * (math.atan2(
                    pos_y - turtlebot_odom.pose.pose.position.y, pos_x -
                    turtlebot_odom.pose.pose.position.x) -
                     2 * math.asin(turtlebot_odom.pose.pose.orientation.z)), 
                      -math.radians(30), math.radians(30))

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

    def set_orientation(desired_theta=0, tol=1e-2):
        ''' turn turtle to a certain orientation '''
        
        # avoid angularity

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
            Kp = 0.1
            Ki = 0.0001
            Kd = 0.003
            # Calculate error
            error = desired_theta - \
                math.degrees(
                    2 * math.asin(turtlebot_odom.pose.pose.orientation.z))

            rospy.loginfo("error= %.3f, theta=%.3f ", error, math.degrees(
                2 * math.asin(turtlebot_odom.pose.pose.orientation.z)))

            if math.fabs(error) < tol:
                break

            # generate vel cmd
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = constrain(
                Kp * error + Ki * i_error + Kd * d_error,
                -math.radians(30),
                math.radians(30))

            velocity_publisher.publish(vel_msg)

            if i_error < 1.0:
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

    def grid_clean_10_by_10():
        '''  '''

        rospy.loginfo("Task: grid clean the room 10 by 10.")
        go_to_pos(-4.0, -4.0)

        for i in range(4):
            set_orientation(0)
            move(0.4, 8, True)
            set_orientation(90)
            move(0.4, 1, True)
            set_orientation(179.8)
            move(0.4, 8, True)
            set_orientation(90)
            move(0.4, 1, True)

        set_orientation(0)
        move(0.4, 10, True)

        print "Grid cleanning is completed!"

    def spiral_clean_10_by_10():
        ''' '''

        print"Start the task: sparial clean the room 10 by 10."

        # Go to the center
        go_to_pos(0.0, 0.0)
        vel_msg = Twist()
        loop_rate = rospy.Rate(1)
        rk = 0.3
        constant_speed = 4
        while not rospy.is_shutdown():
            if turtlebot_odom.pose.pose.position.x >= 4.5 or \
                    turtlebot_odom.pose.pose.position.y >= 4.5:
                break
            vel_msg.linear.x = rk
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = constant_speed

            velocity_publisher.publish(vel_msg)
            rk += 0.3
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
    # Start test
    rospy.loginfo('Task start.')
    rospy.loginfo('Init...')
    # Wait 3 second to make sure gazebo ready
    time.sleep(3)

    #-----------------------------------------------
    # Test move and rotate
    # speed limit 0.4, angular speed limit 30

    # for i in range(10):
    #     rospy.loginfo('Move forward 2 meters...')
    #     move(0.2, 2, True)
    #     rospy.loginfo('Rotate 180 degrees...')
    #     rotate(30, 180, True)
    #     rospy.loginfo('Move forward 2 meters...')
    #     move(0.2, 2, True)
    #     rospy.loginfo('Rotate 180 degrees...')
    #     rotate(30, 180, True)
    #     rospy.loginfo('Move forward 2 meters...')
    #     move(0.2, 2, True)
    #     rospy.loginfo('Rotate 180 degrees...')
    #     rotate(30, 180, True)
    #     rospy.loginfo('Move forward 2 meters...')
    #     move(0.2, 2, True)
    #     rospy.loginfo('Rotate 180 degrees...')
    #     rotate(30, 180, True)

    #----------------------------------------------------
    # Test go_to_pos
    # x limit (-4.5, 4.5), y limit (-4.5, 4.5)

    # rospy.loginfo("Go to position (1,1)")
    # go_to_pos(1, 1)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (3,4)")
    # go_to_pos(3, 4)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (-2,-3)")
    # go_to_pos(-2, -3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(0, 0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    #-------------------------------------------------------
    # Test set_orientation

    # rospy.loginfo("Set orientation to 0 degree")
    # time.sleep(4)
    # set_orientation(0)

    # rospy.loginfo("Set orientation to 150 degree")
    # time.sleep(4)
    # set_orientation(150)

    # rospy.loginfo("Set orientation to -44 degree")
    # time.sleep(4)
    # set_orientation(-44)

    # rospy.loginfo("Set orientation to 90 degree")
    # time.sleep(4)
    # set_orientation(90)

    grid_clean_10_by_10()
    # spiral_clean_10_by_10()

    rospy.loginfo("Task complished.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
