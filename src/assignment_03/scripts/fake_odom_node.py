#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf


def main():
    
    rospy.init_node('fake_odom')
    
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    x = 5.0
    y = -4.0
    th = 0.0

    vx = 0.1
    vy = -0.1
    vth = 0.1
    
    odom_pub = rospy.Publisher("fake_odom_topic",Odometry,queue_size=10);

    loop_rate = rospy.Rate(100)


    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        #print dt
        delta_x = (vx*cos(th) - vy*sin(th))*dt
        delta_y = (vx*sin(th) - vy*cos(th))*dt
        delta_th = vth*dt;
        
        x += delta_x
        y += delta_y
        th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        fake_odom_topic = Odometry()
        fake_odom_topic.header.stamp = current_time
        fake_odom_topic.header.frame_id = "fake_odom"

        # set the position
        fake_odom_topic.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        
        fake_odom_topic.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(fake_odom_topic)

        last_time = current_time
        loop_rate.sleep()

if __name__ == '__main__':
    main()