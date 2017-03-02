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
            'cmd_vel', Twist, queue_size=5)
        self.laser_subscriber = rospy.Subscriber(
            "/mybot/laser/scan", LaserScan, self.laser_callback, queue_size=5)
        self.turtlebot_odom = Odometry()
        self.turtlebot_laser = LaserScan()

        # control frequency
        self.control_frequency = 50 #changing results: 100 is worse, 25 is ok, 10 is bad

    def sort(self, laserscan):
	
        
	closeleft = laserscan.ranges[719]
	farleft = laserscan.ranges[539]
	closeright = laserscan.ranges[0]
	farright = laserscan.ranges[179]
	
	leftangle = math.degrees(math.atan(1-(closeleft/farleft)*(math.sqrt(2)))) #negative angle on left means it is going towards left wall
	rightangle = math.degrees(math.atan(1-(closeright/farright)*(math.sqrt(2)))) #negative angle on right means it is going towards right wall
	self.distfromcenter = closeleft - (closeleft+closeright)/2  # negative is left, positive is right
	self.frontdist = laserscan.ranges[359]
	self.angleerror = leftangle - rightangle

	print("Left angle:" + str(leftangle) + " Right angle:" + str(rightangle) + " Deviation from Center:" + str(self.distfromcenter) + " Front distance:" + str(self.frontdist) )
	

    def movement(self):
        '''Uses the information known about the obstacles to move robot.'''
  	# Define vel_msg
	vel_msg = Twist()

	##################################################################
	#####################     Control System     #####################
	##################################################################
	
	anglegain = 0.01
	centergain = 1

        error = anglegain * self.angleerror - centergain * self.distfromcenter

        if (error < 0.1 and error > -0.1):
             error = 0
      	
	turngain = -0.3
	
	vel_msg.angular.z = turngain * error

	if (self.frontdist < 1):
	     vel_msg.linear.x = 0.2*self.frontdist
	else:
	     vel_msg.linear.x = 0.2

	##################################################################
	#####################     Control System     #####################
	##################################################################
	
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0  
        
        self.velocity_publisher.publish(vel_msg)

    def run_run(self):

        rate = rospy.Rate(self.control_frequency)
        rospy.loginfo("runrun started") 

        while not rospy.is_shutdown():   
            self.movement()
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

    
    def laser_callback(self, laserscan):
        '''Callback function for laser scan '''

        self.sort(laserscan)
        self.movement()

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

