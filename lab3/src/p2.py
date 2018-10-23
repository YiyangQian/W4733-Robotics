#!/usr/bin/env python

""" move_base_square.py - Version 1.1 2013-12-20
    Command a robot to move in a square using move_base actions..
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
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from math import radians, pi

from convex_hull import *

center_shift_array = [[-18,-18],[-18,18],[18,-18],[18,18]]


class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        # How big is the square we want the robot to navigate?
        square_size = rospy.get_param("~square_size", 1.0) # meters

        # Create a list to hold the waypoint poses
        waypoints = list()
        
        total_length = 0

        convex_hull_array = calculate_convex_hull([[-18,-18],[-18,18],[18,-18],[18,18]])
        for i in range(len(convex_hull_array)):
            total_length += convex_hull_array[i].shape[0]
            this_convex_waypoints = list()
            current_convex_hull = convex_hull_array[i]
            print(current_convex_hull)
            for j in range(current_convex_hull.shape[0]):
                new_x = current_convex_hull[j][0]/100
                new_y = current_convex_hull[j][1]/100
                this_convex_waypoints.append(Point(new_x, new_y , 0.0))
            waypoints.append(this_convex_waypoints)

        # Initialize the visualization markers for RViz

        self.init_markers()

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        for i in range(len(convex_hull_array)):
            #self.markers.points = list()
            for j in range(len(waypoints[i]) -1 ):           

                p = waypoints[i][j]
                q = waypoints[i][j+1]
                self.publish_one_edge(p,q)
                
                rospy.sleep(0.1)

            self.publish_one_edge(waypoints[i][-1],waypoints[i][0])
            rospy.sleep(0.1)

        
    def init_markers(self):
        # Set up our waypoint markers
        # Define a marker publisher.
        self.marker_array = MarkerArray()
        self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=20)
        self.marker_id = 0

    def publish_one_edge(self,p1,p2):
        marker_scale = 0.015
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'vgraph'
        
        marker_color = {'r': 0, 'g': 0, 'b': 1, 'a': 1.0}
        
        markers = Marker()
        markers.ns = marker_ns
        markers.id = self.marker_id
        markers.type = Marker.LINE_STRIP
        markers.action = Marker.ADD
        markers.lifetime = rospy.Duration(marker_lifetime)
        markers.scale.x = marker_scale
        markers.scale.y = marker_scale
        markers.color.r = marker_color['r']
        markers.color.g = marker_color['g']
        markers.color.b = marker_color['b']
        markers.color.a = marker_color['a']
        
        markers.header.frame_id = 'odom'
        markers.header.stamp = rospy.Time.now()
        markers.points = list()
        markers.points.append(Point(p1.x,p1.y,0))
        markers.points.append(Point(p2.x,p2.y,0))


        self.marker_array.markers.append(markers)
        self.marker_pub.publish(self.marker_array.markers[-1])
        self.marker_id +=1
        


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")