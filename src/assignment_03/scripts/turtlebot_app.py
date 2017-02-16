#!/usr/bin/env python

"""

"""
import math
import random
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class turtlebot_autonomous:
    ''' '''
    def __init__(self):
        # publishers and subscribers
        self.velocity_publisher = rospy.Publisher(
            'turtle1/cmd_vel', Twist, queue_size=5)
        self.odom_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.odeometry_callback, queue_size=5)
        # self.laser_subscriber = rospy.Subscriber(
        #     "/scan", LaserScan, self.laser_callback, queue_size=5)
        # self.min_distance = 10
        # self.Flag_duck = False
        # self.Flag_turn_left = True
        # self.Flag_arrived_pos = False
        # self.alarm_level = "Green"
        # self.obs_place = "N/A"
        self.turtlebot_odom = Odometry()
        self.turtlebot_laser = LaserScan()

        #init orientation
        self.yaw = 0
        self.t1 = 0
        self.t2 = 0

        # init controller on Yaw
        self.KpY = 3.0
        self.KiY = 0.003
        self.KdY = 0#2.0

        #position variables
        # self.current_x = 0
        # self.current_y = 0

        # # target orientation
        # self.target_orientation = 0


    def odeometry_callback(self, odom_msg):
        ''' call back function of position of turtle'''
        self.turtlebot_odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        self.turtlebot_odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        self.turtlebot_odom.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        self.turtlebot_odom.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        self.turtlebot_odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        self.turtlebot_odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        # # yaw (z-axis rotation)
        self.t1 = 2 * (self.turtlebot_odom.pose.pose.orientation.z * self.turtlebot_odom.pose.pose.orientation.w - self.turtlebot_odom.pose.pose.orientation.x * self.turtlebot_odom.pose.pose.orientation.y)
        self.t2 = 2 * math.pow(self.turtlebot_odom.pose.pose.orientation.x,2) - 1 + 2 * pow(self.turtlebot_odom.pose.pose.orientation.w,2)
        self.yaw = (math.pi + math.atan2(self.t1, self.t2))*180/math.pi

    def constrain(self, val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))

    def orientation_law(self, desired_orientation=0, tol=0.5):
        # define Kp and Ki
        # KpY = 3.0
        # KiY = 0.003
        # KdY = 0#2.0
        # Calculate error
        # t1 = 2 * (turtlebot_odom.pose.pose.orientation.z * turtlebot_odom.pose.pose.orientation.w - turtlebot_odom.pose.pose.orientation.x * turtlebot_odom.pose.pose.orientation.y)
        # t2 = 2 * math.pow(turtlebot_odom.pose.pose.orientation.x,2) - 1 + 2 * pow(turtlebot_odom.pose.pose.orientation.w,2)
        # yaw = (math.pi + math.atan2(t1, t2))*180/math.pi
        gain = 1

        # detect the actual orientation quadrant
        if self.yaw >= 0 and self.yaw < 90:
            qY = 1
        elif self.yaw >= 90 and self.yaw < 180:
            qY = 2
        elif self.yaw >= 180 and self.yaw < 270:
            qY = 3
        else:
            qY = 4

        # detect the desired orientation quadrant
        if desired_orientation >= 0 and desired_orientation < 90:
            qD = 1
        elif desired_orientation >= 90 and desired_orientation < 180:
            qD = 2
        elif desired_orientation >= 180 and desired_orientation < 270:
            qD = 3
        else:
            qD = 4

        # decide in which direction turtlebot has to move
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

            # define the speed of rotation with fixed rate
            zcmd = gain * math.radians(20)
        else:
            # define the speed of rotation using controller
            errorY = (desired_orientation - self.yaw)
            zcmd = self.constrain(
                self.KpY * errorY,
                -math.radians(10),
                math.radians(10))

        rospy.loginfo("yaw=%.2f, zcmd=%.2f, qD=%.2f, qY=%.2f",                      
                self.yaw,
                zcmd, qD, qY)

        return zcmd

    def set_orientation(self, desired_orientation=0, tol=0.5):
        ''' turn turtle to a certain orientation '''

        # Define vel_msg
        vel_msg = Twist()
        # loop rate
        control_frequency = 30
        rate = rospy.Rate(control_frequency)

        while not rospy.is_shutdown():
            # Calculate yaw
            # t1 = 2 * (turtlebot_odom.pose.pose.orientation.z * turtlebot_odom.pose.pose.orientation.w - turtlebot_odom.pose.pose.orientation.x * turtlebot_odom.pose.pose.orientation.y)
            # t2 = 2 * math.pow(turtlebot_odom.pose.pose.orientation.x,2) - 1 + 2 * pow(turtlebot_odom.pose.pose.orientation.w,2)
            # yaw = (math.pi + math.atan2(t1, t2))*180/math.pi

            errorY = (desired_orientation - self.yaw)
            if math.fabs(errorY) < tol:
                break

            zcmd = self.orientation_law(desired_orientation, tol)

            # generate vel cmd
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = zcmd

            self.velocity_publisher.publish(vel_msg)

            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def get_distance(self, x1, y1, x2, y2):
        ''' Calculate distance between (x1, y1) and (x2, y2)'''
        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def go_to_pos(self, pos_x=0.0, pos_y=0.0, tol=0.05):
        ''' Let the turtle go to certain x,y in turtlesim'''

        # get current position
        current_x = self.turtlebot_odom.pose.pose.position.x
        current_y = self.turtlebot_odom.pose.pose.position.y

        angle = math.pi + math.atan2((pos_y - current_y), (pos_x - current_x)) 
        angleD = angle*180/math.pi
        target_orientation = angleD 

        # rospy.loginfo("target_orientation=%.2f, angle=%.2f",                      
        #         target_orientation,
        #         angleD)

        # go to target orientation
        self.set_orientation(target_orientation)
        # stop for 1 second
        time.sleep(1)

        # move straight to desired position.

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

        # init parameters for distance controller
        errorS = 0.0
        last_errorS = 0.0
        d_errorS = 0.0
        i_errorS = 0.0

        while not rospy.is_shutdown():
            # define Kp and Ki
            KpS = 0.3
            KiS = 0.001
            KdS = 1.0
            # Calculate error
            errorS = self.get_distance(
                self.turtlebot_odom.pose.pose.position.x,
                self.turtlebot_odom.pose.pose.position.y,
                pos_x, pos_y)
            d_errorS = (errorS - last_errorS) * control_frequency

            if errorS < tol:
                break

            # generate vel cmd
            vel_msg.linear.x = self.constrain(
                KpS * errorS + KiS * i_errorS + KdS * d_errorS, -0.4, 0.4)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            
            zcmd = self.orientation_law(target_orientation, tol)
            vel_msg.angular.z = zcmd

            self.velocity_publisher.publish(vel_msg)

            if i_errorS < 1:
                i_errorS += errorS / control_frequency
            else:
                i_errorS = 0
            last_errorS = errorS

            rospy.loginfo("x=%.2f, y=%.2f, setx=%.2f, sety=%.2f, setyaw=%.2f",
                      self.turtlebot_odom.pose.pose.position.x,
                      self.turtlebot_odom.pose.pose.position.y,
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
        self.velocity_publisher.publish(vel_msg)

    
def main():

    #####################################################
    # Test code
    #####################################################
    turtlebot = turtlebot_autonomous()
    rospy.init_node('turtlebot_autonomous', anonymous=True)

    # Start test
    rospy.loginfo('Task start.')
    rospy.loginfo('Init...')
    # Wait 3 second to make sure gazebo ready
    time.sleep(1)
   
    #----------------------------------------------------
    # Test go_to_pos
    # x limit (-4.5, 4.5), y limit (-4.5, 4.5)

    rospy.loginfo("Go to position (3,-3)")
    turtlebot.go_to_pos(3, -3)   # go_to_pos(1, 1)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    rospy.loginfo("Go to position (5,-3)")
    turtlebot.go_to_pos(5, -3)   # go_to_pos(1, 1)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    rospy.loginfo("Go to position (5,-1)")
    turtlebot.go_to_pos(5, -1)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    rospy.loginfo("Go to position (0,-1)")
    turtlebot.go_to_pos(0, -1)
    rospy.loginfo("Arrived.")
    time.sleep(2)

    rospy.loginfo("Go to position (0,0)")
    turtlebot.go_to_pos(0, 0)
    rospy.loginfo("Arrived.")
    time.sleep(2)

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
