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
        turtlebot_odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        rospy.loginfo("px=%.2f, py=%.2f, orz= %.3f, theta=%.1f ",
                      turtlebot_odom.pose.pose.position.x,
                      turtlebot_odom.pose.pose.position.y,
                      turtlebot_odom.pose.pose.orientation.w,
                      math.degrees(2 * math.acos(turtlebot_odom.pose.pose.orientation.w)))

    # publisher for cmd_vel to turtlesim
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber(
        "/odom", Odometry, odeometry_callback, queue_size=5)
    rospy.init_node('turtlesim_cleaner', anonymous=True)

    def constrain(val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))

    def set_orientation(desired_orientation_z=0, tol=2e-4):
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
            Kp = 5.0
            Ki = 0.003
            Kd = 2.0
            # Calculate error
            error = desired_orientation_z - turtlebot_odom.pose.pose.orientation.w
            d_error = (error - last_error) * control_frequency

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
                -math.radians(10),
                math.radians(10))

            velocity_publisher.publish(vel_msg)

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
        velocity_publisher.publish(vel_msg)

    def go_to_pos(pos_x=0.0, pos_y=0.0, tol=0.05):
        ''' Let the turtle go to certain x,y in turtlesim'''

        # get current position
        current_x = turtlebot_odom.pose.pose.position.x
        current_y = turtlebot_odom.pose.pose.position.y
        current_w = turtlebot_odom.pose.pose.orientation.w

        # calculate angle
        if pos_y - current_y >= 0:
            temp_angle = math.pi + math.atan2((pos_y - current_y), (pos_x - current_x)) / 2
        else:
            temp_angle = math.atan2((pos_y - current_y), (pos_x - current_x)) / 2

        target_orientation = math.cos(temp_angle)

        # go to target orientation
        set_orientation(target_orientation)
        # stop for 1 second
        time.sleep(1)

        # move straight to desired position.

        # Set limit of x and y
        x_min = -4.5
        x_max = 4.5
        y_min = -4.5
        y_max = 4.5

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

        while not rospy.is_shutdown():
            # define Kp, Ki, Kd
            Kp_distance = 0.3
            Ki_distance = 0.001
            Kd_distance = 1.0

            Kp_orientation = 5.0
            Ki_orientation = 0.003
            Kd_orientation = 2.0

            # Calculate error
            error_distance = get_distance(
                turtlebot_odom.pose.pose.position.x,
                turtlebot_odom.pose.pose.position.y,
                pos_x, pos_y)

            error_orientation = target_orientation - turtlebot_odom.pose.pose.orientation.w

            d_error_distance = (error_distance - last_error_distance) * control_frequency
            d_error_orientation = (error_orientation - \
             last_error_orientation) * control_frequency


            # check stop condition
            if error_distance < tol:
                break

            # generate vel cmd
            vel_msg.linear.x = constrain(
                Kp_distance * error_distance + \
                Ki_distance * i_error_distance + \
                Kd_distance * d_error_distance, -0.35, 0.35)

            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            vel_msg.angular.z = constrain(
                Kp_orientation * error_orientation \
                + Ki_orientation * i_error_orientation + \
                Kd_orientation * d_error_orientation,
                -math.radians(10),
                math.radians(10))

            velocity_publisher.publish(vel_msg)

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
        velocity_publisher.publish(vel_msg)

    def grid_clean_10_by_10():
        '''  '''

        rospy.loginfo("Task: grid clean the room 10 by 10.")
        rospy.loginfo("Init Gazebo world...")
        time.sleep(5)
        
        rospy.loginfo("Go to position (1,1)")
        
        go_to_pos(3, -3)   # go_to_pos(1, 1)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (3,4)")
        go_to_pos(3, 3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (-2,-3)")
        go_to_pos(2, 3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(2, -3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(1, -3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(1, 3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(0, 3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(0, -3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(-1, -3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(-1, 3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(-2, 3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(-2, -3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(-3, -3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(-3, 3)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        rospy.loginfo("Go to position (0,0)")
        go_to_pos(0, 0)
        rospy.loginfo("Arrived.")
        time.sleep(2)

        print "Grid cleanning is completed!"

    #####################################################
    # Test code
    #####################################################
    # Start test
    rospy.loginfo('Task start.')
    rospy.loginfo('Init...')
    # Wait 3 second to make sure gazebo ready
    time.sleep(3)

    #---------------------------------------------------
    # Test set_orientation
    # rospy.loginfo("Set orientation (0.1)")
    # set_orientation(0.1)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (0.2)")
    # set_orientation(0.2)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (0.4)")
    # set_orientation(0.4)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (0.6)")
    # set_orientation(0.6)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (0.8)")
    # set_orientation(0.8)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (1.0)")
    # set_orientation(1.0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (-0.1)")
    # set_orientation(-0.1)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (-0.2)")
    # set_orientation(-0.2)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (-0.4)")
    # set_orientation(-0.4)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (-0.6)")
    # set_orientation(-0.6)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (-0.8)")
    # set_orientation(-0.8)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (-1.0)")
    # set_orientation(-1.0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)




    #----------------------------------------------------
    # Test go_to_pos
    # x limit (-4.5, 4.5), y limit (-4.5, 4.5)

    # rospy.loginfo("Go to position (1,1)")
    # go_to_pos(0, 1)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (3,4)")
    # go_to_pos(1, 1)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (-2,-3)")
    # go_to_pos(1.0, 0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(-1.0, 0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    grid_clean_10_by_10()





    rospy.loginfo("Task complished.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
