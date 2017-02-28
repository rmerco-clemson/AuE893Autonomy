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
        self.laser_subscriber = rospy.Subscriber(
            "/mybot/laser/scan", LaserScan, self.laser_callback, queue_size=5)
        self.turtlebot_odom = Odometry()
        self.turtlebot_laser = LaserScan()

        # from wanderer
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_left = 0    
        self.sect_center = 0  
        self.sect_right = 0  
        self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2}
        self.fwd = {0:.25,1:0,10:0,11:0,100:0.1,101:0,110:0,111:0}
        self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}

        # init parameters for steering controller        
        self.last_errorS = 0.0
        self.d_errorS = 0.0        
        self.i_errorS = 0.0
        self.KpS = 0.9        
        self.KdS = 1.0
        self.KiS = 0.1

        # control frequency
        self.control_frequency = 50
        
    # from wanderer
    def reset_sect(self):
        '''Resets the below variables before each new scan message is read'''
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_center = 0

    # from wanderer
    def sort(self, laserscan):
        '''Goes through 'ranges' array in laserscan message and determines 
        where obstacles are located. The class variables sect_1, sect_2, 
        and sect_3 are updated as either '0' (no obstacles within 0.7 m)
        or '1' (obstacles within 0.7 m)

        Parameter laserscan is a laserscan message.'''
        entries = len(laserscan.ranges)
        self.sect_left = laserscan.range_max            
        self.sect_center = laserscan.range_max         
        self.sect_right = laserscan.range_max         
        
        for entry in range(0,entries):
            # if 0.4 < laserscan.ranges[entry] < 0.75:
            if 0.2 < laserscan.ranges[entry] < 0.65:
                self.sect_1 = 1 if (0 < entry < entries/3) else 0 
                self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
                self.sect_3 = 1 if (entries/2 < entry < entries) else 0

            # left minimum distance sensed
            if (0 < entry < entries*2/5):
                if (laserscan.ranges[entry] < self.sect_left):
                    self.sect_left = laserscan.ranges[entry]

            # center minimum distance sensed
            if (entries*2/5 < entry < entries*3/5):
                if (laserscan.ranges[entry] < self.sect_center):
                    self.sect_center = laserscan.ranges[entry]

            # right minimum distance sensed
            if (entries*3/5 < entry < entries):
                if (laserscan.ranges[entry] < self.sect_right):
                    self.sect_right = laserscan.ranges[entry]

        rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3) + " sect_center: " + str(self.sect_center))

    def movement(self, sect1, sect2, sect3):
        '''Uses the information known about the obstacles to move robot.'''
        sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
        rospy.loginfo("Sect = " + str(sect))     

        # Calculate error
        errorST = self.sect_right - self.sect_left
        if (math.fabs(errorST) > 0 ):
            errorS = errorST
        else:
            errorS = 0            
        self.d_errorS = (errorS - self.last_errorS) / self.control_frequency
        self.last_errorS = errorS
        self.i_errorS += errorS * self.control_frequency

        rospy.loginfo("Error = " + str(errorS))

        # Define vel_msg
        vel_msg = Twist()

        # self.KiS * self.i_errorS +
        if (sect == 0):
            vel_msg.angular.z = self.constrain(self.KpS * errorS + self.KdS * self.d_errorS, -0.5, 0.5)
            vel_msg.linear.x = self.constrain(self.sect_center * 0.3, 0.0, 0.5)
        else:
            vel_msg.angular.z = self.ang[sect]
            vel_msg.linear.x = self.fwd[sect]      
        
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        self.velocity_publisher.publish(vel_msg)

        self.reset_sect()

    def run_run(self):

        rate = rospy.Rate(self.control_frequency)
        rospy.loginfo("runrun started") 

        while not rospy.is_shutdown():
            # rospy.loginfo("while movement")     
            self.movement(self.sect_1, self.sect_2, self.sect_3)
            rate.sleep()

        # Define vel_msg
        vel_msg = Twist()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        

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
    
    def laser_callback(self, laserscan):
        '''Callback function for laser scan '''

        self.sort(laserscan)
        self.movement(self.sect_1, self.sect_2, self.sect_3)

    def constrain(self, val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))

    def go_to_pos(self, pos_x=0.0, pos_y=0.0, tol=0.05):
        ''' Let the turtle go to certain x,y in turtlesim'''

        # Reset Flag arrived pos
        self.Flag_arrived_pos = False

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
        #self.set_orientation(target_orientation)
        # stop for 1 second
        #time.sleep(1)

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

        while self.Flag_duck is False and self.Flag_arrived_pos is False and not rospy.is_shutdown():
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
                self.Flag_arrived_pos = True
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

            # rospy.loginfo("x=%.2f, y=%.2f, setx=%.2f, sety=%.2f, setyaw=%.2f",
            #           self.turtlebot_odom.pose.pose.position.x,
            #           self.turtlebot_odom.pose.pose.position.y,
            #           pos_x,
            #           pos_y,
            #           target_orientation)
        
            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
def main():

    # Start
    rospy.loginfo('Task start.')
    rospy.loginfo('Init...')

    turtlebot = turtlebot_autonomous()
    rospy.init_node('racing_autonomous', anonymous=True)

    rospy.spin()
    # turtlebot.run_run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
