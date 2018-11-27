#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
        Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
        Twist, queue_size=1)
        self.twist = Twist()
        self.r = rospy.Rate(1)
        #cv2.namedWindow("window", 1)
    
    def image_callback(self, msg):
        #print("inside image_callback")
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 26, 43, 46])
        upper_yellow = numpy.array([34, 255, 250])

        lower_blue = numpy.array([100, 43, 46])
        upper_blue = numpy.array([124, 255, 250])

        lower_green = numpy.array([35, 43, 46])
        upper_green = numpy.array([77, 255, 250])

        lower_green = numpy.array([35, 43, 46])
        upper_green = numpy.array([77, 255, 250])

        lower_red = numpy.array([156, 43, 46])
        upper_red = numpy.array([180, 255, 250])

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        yellow_mask[0:search_top, 0:w] = 0
        yellow_mask[search_bot:h, 0:w] = 0
        blue_mask[0:search_top, 0:w] = 0
        blue_mask[search_bot:h, 0:w] = 0
        green_mask[0:search_top, 0:w] = 0
        green_mask[search_bot:h, 0:w] = 0
        red_mask[0:search_top, 0:w] = 0
        red_mask[search_bot:h, 0:w] = 0

        M_yellow = cv2.moments(yellow_mask)
        M_blue = cv2.moments(blue_mask)
        M_green = cv2.moments(green_mask)
        M_red = cv2.moments(red_mask)

        if M_red['m00']>0:
            print("stop")
            exit(0)

        elif M_blue['m00']>0:
            print("blue detected!")
            self.twist.linear.x = 0.5
            self.twist.angular.z = -0.3
            self.cmd_vel_pub.publish(self.twist)

        elif M_green['m00']>0:
            print("green detected!")
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)

        elif M_yellow['m00'] > 0:
            cx = int(M_yellow['m10']/M_yellow['m00'])
            cy = int(M_yellow['m01']/M_yellow['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
