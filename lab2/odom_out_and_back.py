#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20
    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import sys
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan

from sensor_msgs.msg import LaserScan

class Lab2():
    def __init__(self):
        self.g_range_ahead = 1
        self.saved_pos = []
        self.rotate_flag = False
        # Give the node a name
        rospy.init_node('lab2', anonymous=False)

        # Connect the node with sensor
        scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)


        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # How fast will we update the robot's movement?
        rate = 5
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        linear_speed = 0.4

        # Set the rotation speed in radians per second
        angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  


        # Initialize the position variable as a Point type
        position = Point()

        destination = Point(10, 0, 0)
        
        is_following_m_line = True

        while(self.getDistance(position, destination) > 0.15):
            # print("entering while loop")
            # print("outside cases", self.g_range_ahead)

            if is_following_m_line:
                # Set the movement command to forward motion
                # Initialize the movement command
                move_cmd = Twist()
                move_cmd.linear.x = linear_speed
            
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                (position, rotation) = self.get_odom()
                if self.g_range_ahead < 1.3:
                    is_following_m_line = False
                    
                    self.saved_pos.append(position)

            else:
                move_cmd = Twist()
                if self.g_range_ahead < 1.3:
                    # self.rotateTillNotBlocked(angular_speed)
                    move_cmd = Twist()
                    move_cmd.angular.z = angular_speed
                    
                    for i in range(3):
                        self.cmd_vel.publish(move_cmd)
                        r.sleep()
                    self.rotate_flag = True
                    (position, rotation) = self.get_odom()
                elif self.g_range_ahead > 2 and not self.rotate_flag:
                    # right turning 
                    print("we have entered right turn")
                    move_cmd.angular.z = -1 * angular_speed
                    
                    for i in range(3):
                        self.cmd_vel.publish(move_cmd)
                        r.sleep()
                    (position, rotation) = self.get_odom()
                else:
                    move_cmd.linear.x = linear_speed
                    
                    for i in range(5):
                        self.cmd_vel.publish(move_cmd)
                        r.sleep()
                    (position, rotation) = self.get_odom()
                    self.rotate_flag = False
                    
                    if abs(position.y)<0.3:
                        print("reached M line!!!")
                        have_been_here = False
                        for i in range(len(self.saved_pos)):
                            print(i,self.saved_pos[i])
                            if getDistance(position,self.saved_pos[i]) < 0.3:
                                have_been_here = True
                                break
                        print( "current have been here flag is: ", have_been_here )
                        if not have_been_here:
                            self.saved_pos.append(position)

                            is_following_m_line = True
                            print("status back to going straight in M line")
    
    def getDistance(self, position, destination):
        print(position.x, position.y)
        result =  sqrt((position.x- destination.x)**2+ ( position.y-destination.y)**2)
        return result
        

    def scan_callback(self, msg):
        min_dis = sys.float_info.max
        for i in msg.ranges:
            if not isnan(i):
                min_dis = min(min_dis, i)
        self.g_range_ahead = min_dis
        print(min_dis)

        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Lab2()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
