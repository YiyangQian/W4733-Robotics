#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):
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
        linear_speed = 0.2
        
        # Set the rotation speed to 1.0 radians per second
        angular_speed = 1.0


        while True: 
            instruc = raw_input("Please Enter T or R or Q:  ")
            print(instruc)
            if instruc is "T":
                linear_speed, linear_duration = self.getTParams(linear_speed)
                self.translate(linear_speed, linear_duration, rate, r)
            elif instruc is "R":
                angular_speed, angular_duration = self.getRParams(angular_speed)
                self.rotate(angular_speed, angular_duration, rate, r)
            elif instruc is "Q":
                self.cmd_vel.publish(Twist())
                break
            else:
                print("Please Entern a Valid Input")
        
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
        OutAndBack()
    except Exception as e:
        rospy.loginfo("Out-and-Back node terminated.")
        print(e)
