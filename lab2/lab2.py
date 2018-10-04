#!/usr/bin/env python

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
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=200)
        
        # How fast will we update the robot's movement?
        rate = 5
        
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        self.linear_speed = 0.4

        # Set the rotation speed in radians per second
        angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(5.0)

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
        
        # flag of state: following-m-line or going-around-obstacle
        is_following_m_line = True

        while(self.getDistance(position, destination) > 0.3):
            if is_following_m_line:
                print("moving forward")
                self.moveForward(1)

                (position, rotation) = self.get_odom()
 
                if self.g_range_ahead < 1.3:
                    is_following_m_line = False
                    # save the position of leaving m-line
                    self.saved_pos.append(position)

            else:
                if self.g_range_ahead < 1.3:
                    # left turning
                    print("turning left")
                    self.rotate(angular_speed, 3)
                    self.rotate_flag = True
                    (position, rotation) = self.get_odom()
                
                elif self.g_range_ahead > 2 and not self.rotate_flag:
                    # right turning 
                    print("turning right")
                    self.rotate(-1 * angular_speed, 3)
                    (position, rotation) = self.get_odom()
                
                else:
                    # after turn left, must move forward; current distance to object is between 1.3 and 2.0
                    print("moving forward")
                    self.moveForward(5)
                    (position, rotation) = self.get_odom()
                    self.rotate_flag = False
                    
                    if abs(position.y) < 0.2:
                        print("reached M line")
                        have_been_here = False
                        we_are_closer = True
                        current_position_dis = self.getDistance(position,destination)
                        
                        for i in range(len(self.saved_pos)):
                            curpos = self.saved_pos[i]
                            tmp_dis = self.getDistance(position,curpos)
                            tmp_dest_dis = self.getDistance(destination,curpos)
                            if tmp_dest_dis < current_position_dis:
                                we_are_closer = False
                                break
                            if  tmp_dis < 0.5:
                                have_been_here = True
                                print("I have been to this place")
                                if i == 0 and len(self.saved_pos) > 1:
                                    print("Noob, you cannot solve this")
                                    self.shutdown()
                                    return
                                break
                        print( "current have been here flag is: ", have_been_here )
                        print ("Are we closer? ", we_are_closer)
                        if (not have_been_here) and we_are_closer:
                            self.saved_pos.append(position)
                            is_following_m_line = True
                            print("status back to going straight in M line")

                            # turn around to head to destination                            
                            while abs(rotation) > angular_tolerance:
                                if abs(rotation)< angular_tolerance:
                                    break
                                else:
                                    self.rotate(angular_speed, 1)
                                    (position, rotation) = self.get_odom()

    def rotate(self, angular_speed, steps):
        move_cmd = Twist()
        move_cmd.angular.z = angular_speed
        for i in range(steps):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

    def moveForward(self, steps):
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        for i in range(steps):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
    
    def getDistance(self, position, destination):
        result =  sqrt((position.x- destination.x)**2+ ( position.y-destination.y)**2)
        return result
        
    def scan_callback(self, msg):
        min_dis = sys.float_info.max
        for i in msg.ranges:
            if not isnan(i):
                min_dis = min(min_dis, i)
        self.g_range_ahead = min_dis
      
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
    except Exception as e:
        print(e)
        rospy.loginfo("Lab2 node terminated.")