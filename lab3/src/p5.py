import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from convex_hull import *
from p4 import *

import sys
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan, hypot
from sensor_msgs.msg import LaserScan

class Move():
    def __init__(self, move_arr):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # How fast will we update the robot's movement?
        rate = 50
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.2 meters per second 
        self.linear_speed = 0.2
        
        # Set the rotation speed to 1.0 radians per second
        self.angular_speed = 0.5
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

        for i in range(1,len(move_arr)):
            self.move(move_arr[i][0], move_arr[i][1], rate, r)


    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def move(self, dst_x, dst_y, rate, r):
        (position, rotation) = self.get_odom()

        dy = dst_y - position.y
        dx = dst_x - position.x

        dst_angle = np.arctan(dy/dx)
        if np.sign(dx)<0:
            dst_angle += pi
        print(position, rotation, dst_angle)

        clock_wise_rotate = dst_angle - rotation
        anti_clock_rotate = rotation - dst_angle
        if abs(clock_wise_rotate)> abs(anti_clock_rotate):
            goal_angle = anti_clock_rotate
        else:
            goal_angle = clock_wise_rotate

        self.angular_speed = abs(self.angular_speed)
        if goal_angle < 0: 
            self.angular_speed = -self.angular_speed
        angular_duration = goal_angle / self.angular_speed
        self.rotate(self.angular_speed, angular_duration, rate, r)

        self.linear_speed = abs(self.linear_speed)
        goal_distance = float( np.hypot(dx,dy) )
        if goal_distance < 0:
            self.linear_speed = -self.linear_speed
        linear_duration = goal_distance / self.linear_speed
        self.translate(self.linear_speed, linear_duration, rate, r)

        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        
    def getDistance(self, position, destination):
        result =  sqrt((position.x- destination.x)**2+ ( position.y-destination.y)**2)
        return result

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
    
    def stop(self):
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
    
    def translate(self, linear_speed, linear_duration, rate, r):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        ticks = int(linear_duration * rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        self.stop()
    
    def rotate(self, angular_speed, angular_duration, rate, r):
        move_cmd = Twist()
        move_cmd.angular.z = angular_speed
        ticks = int(angular_duration * rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        self.stop()
    
    def getTParams(self, linear_speed):
        linear_speed = abs(linear_speed)
        goal_distance = float(raw_input("Please Enter Goal Distance: "))
        if goal_distance < 0:
            linear_speed = -linear_speed
        linear_duration = goal_distance / linear_speed
        return linear_speed, linear_duration
    
    def getRParams(self, angular_speed):
        angular_speed = abs(angular_speed)
        goal_angle = float(raw_input("Please Enter Goal Angular: "))
        if goal_angle < 0: 
            angular_speed = -angular_speed
        angular_duration = goal_angle / angular_speed
        return angular_speed, angular_duration

if __name__ == '__main__':
    try:
        # p4 = P4(False)
        # move_array = p4.map_position_array
        # p4.shutdown()

        move_array = [(0, 0), (0.81999999999999995, 0.68000000000000005), (1.1799999999999999, 0.68000000000000005), (2.9300000000000002, 0.42999999999999999), (4.3200000000000003, 0.12), (6, 0)]
        print(move_array)
        Move(move_array)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
