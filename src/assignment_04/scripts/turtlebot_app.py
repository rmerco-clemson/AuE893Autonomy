#!/usr/bin/env python

"""

"""
import math
import random
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class turtlebot_autonomous:
    ''' '''
    def __init__(self):
        # publishers and subscribers
        self.velocity_publisher = rospy.Publisher(
            'turtle1/cmd_vel', Twist, queue_size=5)
        self.laser_subscriber = rospy.Subscriber(
            "/mybot/laser/scan", LaserScan, self.laser_callback, queue_size=5)
        self.turtlebot_laser = LaserScan()

        # from wanderer
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2}
        self.fwd = {0:.25,1:0,10:0,11:0,100:0.1,101:0,110:0,111:0}
        self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}

        # min distances in the three sectors
        self.sect_left = 0    
        self.sect_center = 0  
        self.sect_right = 0  
        
        # init parameters for steering controller    
        self.KpS = 0.9        
        self.KdS = 1.0    
        self.last_errorS = 0.0
        self.d_errorS = 0.0       

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
            # if 0.4 < laserscan.ranges[entry] < 0.75: # original laser scan ranges
            if 0 < laserscan.ranges[entry] < 0.35:
                self.sect_1 = 1 if (0 < entry < entries/3) else 0 
                self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
                self.sect_3 = 1 if (entries/2 < entry < entries) else 0

            # identify the minimum distance for each sector
            # added to wanderer: left minimum distance sensed            
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

        #rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3) + " sect_center: " + str(self.sect_center))

    def movement(self, sect1, sect2, sect3):
        '''Uses the information known about the obstacles to move robot.'''
        sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))

        # Calculate error for PD
        errorST = self.sect_right - self.sect_left

        # dead band
        if (math.fabs(errorST) > 0.1 ): 
            errorS = errorST
        else:
            errorS = 0            
        
        # error calculation
        self.d_errorS = (errorS - self.last_errorS) / self.control_frequency
        self.last_errorS = errorS        

        # debug
        rospy.loginfo("Sect = " + str(sect) + " - sect_center: " + str(self.sect_center) + " - Error = " + str(errorS))     

        # Define vel_msg
        vel_msg = Twist()

        # logic: if no wanderer sector is involved, PD logics - otherwise wanderer
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
   
    
    def laser_callback(self, laserscan):
        '''Callback function for laser scan '''

        self.sort(laserscan)
        self.movement(self.sect_1, self.sect_2, self.sect_3)

    def constrain(self, val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))
    
def main():

    # Start
    rospy.loginfo('Task start.')
    rospy.loginfo('Init...')

    turtlebot = turtlebot_autonomous()
    rospy.init_node('racing_autonomous', anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
