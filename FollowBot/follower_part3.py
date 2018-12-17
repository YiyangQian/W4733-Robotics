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
        self.flag = 0
        self.left_rgb = cv2.imread('./template/template_left_rgb_masked.jpg')
        self.right_rgb = cv2.imread('./template/template_right_rgb_masked.jpg')
        self.star_rgb = cv2.imread('./template/template_star_rgb_masked.jpg')

        lower_red = numpy.array([0, 43, 46])
        upper_red = numpy.array([20, 255, 250])

        self.left = cv2.inRange( cv2.cvtColor(self.left_rgb, cv2.COLOR_BGR2HSV), lower_red, upper_red)
        self.right = cv2.inRange( cv2.cvtColor(self.right_rgb, cv2.COLOR_BGR2HSV), lower_red, upper_red)
        self.star = cv2.inRange( cv2.cvtColor(self.star_rgb, cv2.COLOR_BGR2HSV), lower_red, upper_red)


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

        lower_red = numpy.array([0, 43, 46])
        upper_red = numpy.array([20, 255, 250])

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask_original = cv2.inRange(hsv, lower_red, upper_red)

        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        h, w, d = image.shape
        search_top = 7*h/8-15
        search_bot = 7*h/8
        yellow_mask[0:search_top, 0:w] = 0
        yellow_mask[search_bot:h, 0:w] = 0
        red_mask[0:search_top, 0:w] = 0
        red_mask[search_bot:h, 0:w] = 0

        M_yellow = cv2.moments(yellow_mask)
        M_red = cv2.moments(red_mask)

        if self.flag == 0:
            if M_red['m00']>0:

                left_res = cv2.matchTemplate(red_mask_original, self.left, cv2.TM_CCOEFF_NORMED)
                right_res = cv2.matchTemplate(red_mask_original, self.right, cv2.TM_CCOEFF_NORMED)
                star_res = cv2.matchTemplate(red_mask_original, self.star, cv2.TM_CCOEFF_NORMED)
                th = 0.7
                left_max = numpy.max(left_res)
                rigth_max = numpy.max(right_res)
                start_max = numpy.max(star_res)
                print(left_max, rigth_max, start_max)
                if left_max > th:
                    print('left')
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0.07
                    self.cmd_vel_pub.publish(self.twist)
                elif rigth_max > th-0.05:
                    print('right')
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = -0.12
                    self.cmd_vel_pub.publish(self.twist)
                elif start_max > 0.64:
                    print('star')
                    self.flag = 40
                else:
                    self.twist.linear.z = 0
                    self.cmd_vel_pub.publish(self.twist)

            elif M_yellow['m00'] > 0:
                cx = int(M_yellow['m10']/M_yellow['m00'])
                cy = int(M_yellow['m01']/M_yellow['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)

        elif self.flag>1:
            self.cmd_vel_pub.publish(self.twist)
            self.flag -= 1 
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("window", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
