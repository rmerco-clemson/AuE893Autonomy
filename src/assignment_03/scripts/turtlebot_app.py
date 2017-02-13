#!/usr/bin/env python

"""

"""
import math
import random
import time
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion


def main():
    ''' '''

    turtlebot_odom = Odometry()
    yaw = 0

    def odeometry_callback(odom_msg):
        ''' call back function of position of turtle'''
        turtlebot_odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        turtlebot_odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        turtlebot_odom.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        turtlebot_odom.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        turtlebot_odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        turtlebot_odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        # quaternion = (
        #     turtlebot_odom.pose.pose.orientation.x,
        #     turtlebot_odom.pose.pose.orientation.y,
        #     turtlebot_odom.pose.pose.orientation.z,
        #     turtlebot_odom.pose.pose.orientation.w)
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]

        # (roll, pitch, yaw) = euler_from_quaternion([
        #     turtlebot_odom.pose.pose.orientation.x, 
        #     turtlebot_odom.pose.pose.orientation.y, 
        #     turtlebot_odom.pose.pose.orientation.z, 
        #     turtlebot_odom.pose.pose.orientation.w])

        # # yaw (z-axis rotation)
        t1 = 2 * (turtlebot_odom.pose.pose.orientation.z * turtlebot_odom.pose.pose.orientation.w - turtlebot_odom.pose.pose.orientation.x * turtlebot_odom.pose.pose.orientation.y)
        t2 = 2 * math.pow(turtlebot_odom.pose.pose.orientation.x,2) - 1 + 2 * pow(turtlebot_odom.pose.pose.orientation.w,2)
        yaw = (math.pi + math.atan2(t1, t2))*180/math.pi


        # rospy.loginfo("px=%.2f, py=%.2f, oz=%.2f, ow=%.2f, yaw=%.2f, yaw2=%.2f",
        #               turtlebot_odom.pose.pose.position.x,
        #               turtlebot_odom.pose.pose.position.y,
        #               turtlebot_odom.pose.pose.orientation.z,
        #               turtlebot_odom.pose.pose.orientation.w,
        #               math.degrees(2 * math.acos(turtlebot_odom.pose.pose.orientation.w)),
        #               yaw)
                    #   t4,
                    #   tf::getYaw(turtlebot_odom.pose.pose.orientation),
                    #   turtlebot_odom.pose.pose.orientation.w)
                    #   math.degrees(2 * math.acos(turtlebot_odom.pose.pose.orientation.w)))

    # publisher for cmd_vel to turtlesim
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber(
        "/odom", Odometry, odeometry_callback, queue_size=5)
    rospy.init_node('turtlesim_cleaner', anonymous=True)

    def constrain(val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))

    def orientation_law(desired_orientation=0, tol=0.5):
        # define Kp and Ki
        KpY = 5.0
        KiY = 0.003
        KdY = 0#2.0
        # Calculate error
        t1 = 2 * (turtlebot_odom.pose.pose.orientation.z * turtlebot_odom.pose.pose.orientation.w - turtlebot_odom.pose.pose.orientation.x * turtlebot_odom.pose.pose.orientation.y)
        t2 = 2 * math.pow(turtlebot_odom.pose.pose.orientation.x,2) - 1 + 2 * pow(turtlebot_odom.pose.pose.orientation.w,2)
        yaw = (math.pi + math.atan2(t1, t2))*180/math.pi
        gain = 1

        if yaw >= 0 and yaw < 90:
            qY = 1
        elif yaw >= 90 and yaw < 180:
            qY = 2
        elif yaw >= 180 and yaw < 270:
            qY = 3
        else:
            qY = 4

        if desired_orientation >= 0 and desired_orientation < 90:
            qD = 1
        elif desired_orientation >= 90 and desired_orientation < 180:
            qD = 2
        elif desired_orientation >= 180 and desired_orientation < 270:
            qD = 3
        else:
            qD = 4

        if qY != qD:
            if qD == 1: # first quadrant
                if qY == 2:
                    gain = -1   # clockwise
                elif qY == 3:
                    gain = -1
                elif qY == 4:
                    gain = 1   # counter clockwise
            elif qD == 2: # second quadrant
                if qY == 1:
                    gain = 1
                elif qY == 3:
                    gain = -1
                elif qY == 4:
                    gain = -1
            elif qD == 3: # third quadrant
                if qY == 1:
                    gain = 1
                elif qY == 2:
                    gain = 1
                elif qY == 4:
                    gain = -1
            elif qD == 4: # fourth quadrant
                if qY == 1:
                    gain = -1
                elif qY == 2:
                    gain = 1
                elif qY == 3:
                    gain = 1
            # else:
            #     gain = -1
            zcmd = gain * math.radians(20)
        else:
            errorY = (desired_orientation - yaw)
            zcmd = constrain(
                KpY * errorY,
                -math.radians(10),
                math.radians(10))

        rospy.loginfo("yaw=%.2f, zcmd=%.2f, qD=%.2f, qY=%.2f",                      
                yaw,
                zcmd, qD, qY)

        return zcmd

    def set_orientation(desired_orientation=0, tol=0.5):
        ''' turn turtle to a certain orientation '''

        # avoid angularity

        # Define vel_msg
        vel_msg = Twist()
        # loop rate
        control_frequency = 30
        rate = rospy.Rate(control_frequency)

        while not rospy.is_shutdown():
            # # define Kp and Ki
            # KpY = 5.0
            # KiY = 0.003
            # KdY = 0#2.0
            # Calculate yaw
            t1 = 2 * (turtlebot_odom.pose.pose.orientation.z * turtlebot_odom.pose.pose.orientation.w - turtlebot_odom.pose.pose.orientation.x * turtlebot_odom.pose.pose.orientation.y)
            t2 = 2 * math.pow(turtlebot_odom.pose.pose.orientation.x,2) - 1 + 2 * pow(turtlebot_odom.pose.pose.orientation.w,2)
            yaw = (math.pi + math.atan2(t1, t2))*180/math.pi
            # gain = 1

            # if yaw >= 0 and yaw < 90:
            #     qY = 1
            # elif yaw >= 90 and yaw < 180:
            #     qY = 2
            # elif yaw >= 180 and yaw < 270:
            #     qY = 3
            # else:
            #     qY = 4

            # if desired_orientation >= 0 and desired_orientation < 90:
            #     qD = 1
            # elif desired_orientation >= 90 and desired_orientation < 180:
            #     qD = 2
            # elif desired_orientation >= 180 and desired_orientation < 270:
            #     qD = 3
            # else:
            #     qD = 4

            # if qY != qD:
            #     if qD == 1: # first quadrant
            #         if qY == 2:
            #             gain = -1   # clockwise
            #         elif qY == 3:
            #             gain = -1
            #         elif qY == 4:
            #             gain = 1   # counter clockwise
            #     elif qD == 2: # second quadrant
            #         if qY == 1:
            #             gain = 1
            #         elif qY == 3:
            #             gain = -1
            #         elif qY == 4:
            #             gain = -1
            #     elif qD == 3: # third quadrant
            #         if qY == 1:
            #             gain = 1
            #         elif qY == 2:
            #             gain = 1
            #         elif qY == 4:
            #             gain = -1
            #     elif qD == 4: # fourth quadrant
            #         if qY == 1:
            #             gain = -1
            #         elif qY == 2:
            #             gain = 1
            #         elif qY == 3:
            #             gain = 1
            #     # else:
            #     #     gain = -1
            #     zcmd = gain * math.radians(20)
            # else:
            #     errorY = (desired_orientation - yaw)
            #     zcmd = constrain(
            #         KpY * errorY + KiY * i_errorY + KdY * d_errorY,
            #         -math.radians(10),
            #         math.radians(10))

            errorY = (desired_orientation - yaw)
            if math.fabs(errorY) < tol:
                break

            zcmd = orientation_law(desired_orientation, tol)

            # generate vel cmd
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = zcmd

            velocity_publisher.publish(vel_msg)

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

        # calculate angle
        #target_orientation = math.cos(math.atan2((pos_y - current_y), (pos_x - current_x)) / 2)
        if (pos_y - current_y)>=0:
            angle = math.pi + math.atan2((pos_y - current_y), (pos_x - current_x)) / 2
        else:
            angle = math.atan2((pos_y - current_y), (pos_x - current_x)) / 2
        target_orientation = math.cos(angle)

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
        errorS = 0.0
        last_errorS = 0.0
        d_errorS = 0.0
        i_errorS = 0.0

        # orientation control
        errorY = 0.0
        last_errorY = 0.0
        d_errorY = 0.0
        i_errorY = 0.0

        while not rospy.is_shutdown():
            # define Kp and Ki
            KpS = 0.3
            KiS = 0.001
            KdS = 1.0
            # Calculate error
            errorS = get_distance(
                turtlebot_odom.pose.pose.position.x,
                turtlebot_odom.pose.pose.position.y,
                pos_x, pos_y)
            d_errorS = (errorS - last_errorS) * control_frequency

            # define Kp and Ki orientation
            KpY = 5.0
            KiY = 0.003
            KdY = 2.0
            # Calculate error orientation
            errorY = target_orientation - turtlebot_odom.pose.pose.orientation.w
            d_errorY = (errorY - last_errorY) * control_frequency

            if errorS < tol:
                break

            # generate vel cmd
            vel_msg.linear.x = constrain(
                KpS * errorS + KiS * i_errorS + KdS * d_errorS, -0.4, 0.4)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            vel_msg.angular.z = constrain(
                KpY * errorY + KiY * i_errorY + KdY * d_errorY,
                -math.radians(10),
                math.radians(10))

            velocity_publisher.publish(vel_msg)

            if i_errorS < 1:
                i_errorS += errorS / control_frequency
            else:
                i_errorS = 0
            last_errorS = errorS

            if i_errorY < 1.5:
                i_errorY += errorY / control_frequency
            else:
                i_errorY = 0
            last_errorY = errorY

            rospy.loginfo("setx=%.2f, sety=%.2f, setyaw=%.2f",
                      pos_x,
                      pos_y,
                      target_orientation)
        
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

        go_to_pos(-3.0, -3.0)
        time.sleep(1)
        go_to_pos(-3.0, 3.0)
        time.sleep(1)
        go_to_pos(-2.0, 3.0)
        time.sleep(1)
        go_to_pos(-2.0, -3.0)
        time.sleep(1)



        print "Grid cleanning is completed!"

    #####################################################
    # Test code
    #####################################################
    # Start test
    rospy.loginfo('Task start.')
    rospy.loginfo('Init...')
    # Wait 3 second to make sure gazebo ready
    time.sleep(1)

    #---------------------------------------------------
    #Test set_orientation
    rospy.loginfo("Set orientation (0)")
    set_orientation(0)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    set_orientation(180)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    # rospy.loginfo("Set orientation (0.5)")
    set_orientation(90)
    rospy.loginfo("Arrived.")
    time.sleep(2)
    
    set_orientation(360)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    set_orientation(270)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    

    # rospy.loginfo("Set orientation (1)")
    # set_orientation(1.0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (0)")
    # set_orientation(-1.0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (0.5)")
    # set_orientation(-0.5)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Set orientation (1)")
    # set_orientation(0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)



    #----------------------------------------------------
    # Test go_to_pos
    # x limit (-4.5, 4.5), y limit (-4.5, 4.5)

    # rospy.loginfo("Go to position (1,1)")
    # go_to_pos(3, -3)   # go_to_pos(1, 1)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (3,4)")
    # go_to_pos(3, 3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (-2,-3)")
    # go_to_pos(2, 3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(2, -3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(1, -3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(1, 3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(0, 3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(0, -3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(-1, -3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(-1, 3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(-2, 3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(-2, -3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(-3, -3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(-3, 3)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)

    # rospy.loginfo("Go to position (0,0)")
    # go_to_pos(0, 0)
    # rospy.loginfo("Arrived.")
    # time.sleep(2)


    # # grid_clean_10_by_10()
    # # spiral_clean_10_by_10()

    # rospy.loginfo("Task complished.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
