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
        # cv2.namedWindow("transfered", 2)
        self.flag = 0
        self.count = 0
        self.left_rgb = cv2.imread('./template/template_left_rgb_masked.jpg')
        self.right_rgb = cv2.imread('./template/template_right_rgb_masked.jpg')
        self.star_rgb = cv2.imread('./template/template_star_rgb_masked.jpg')

        lower_red = numpy.array([0, 43, 46])
        upper_red = numpy.array([20, 255, 250])

        self.frame_count = 0
        self.left = cv2.inRange( cv2.cvtColor(self.left_rgb, cv2.COLOR_BGR2HSV), lower_red, upper_red)
        self.right = cv2.inRange( cv2.cvtColor(self.right_rgb, cv2.COLOR_BGR2HSV), lower_red, upper_red)
        self.star = cv2.inRange( cv2.cvtColor(self.star_rgb, cv2.COLOR_BGR2HSV), lower_red, upper_red)


    def image_callback(self, msg):
        #print("inside image_callback")
        # image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # lower_yellow = numpy.array([ 10, 10, 10])
        # upper_yellow = numpy.array([255, 255, 250])

        # yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # yellow_mask_original = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # h, w, d = image.shape
        # search_top = 7*h/8-15
        # search_bot = 7*h/8
        # yellow_mask[0:search_top, 0:w] = 0
        # yellow_mask[search_bot:h, 0:w] = 0

        # M_yellow = cv2.moments(yellow_mask)

        self.frame_count +=1
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])

        lower_yellow2 = numpy.array([ 30, 10, 10])
        upper_yellow2 = numpy.array([180, 255, 250])

        yellow_mask_original = cv2.inRange(hsv, lower_yellow2, upper_yellow2)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
        search_top = 7*h/8 -15 
        search_bot = 7*h/8
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)

        #print(mask is yellow_mask_original)

        if self.flag == 0:
            if M['m00']>0:

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100
                start_max, left_max, rigth_max =0,0,0

                self.frame_count+=1
                if self.frame_count<1800:
                    left_res = cv2.matchTemplate(yellow_mask_original, self.left, cv2.TM_CCOEFF_NORMED)
                    right_res = cv2.matchTemplate(yellow_mask_original, self.right, cv2.TM_CCOEFF_NORMED)
                    left_max = numpy.max(left_res)
                    rigth_max = numpy.max(right_res)
                else:
                    star_res = cv2.matchTemplate(yellow_mask_original, self.star, cv2.TM_CCOEFF_NORMED)
                    start_max = numpy.max(star_res)
                    print(start_max)

                th = 0.7
                
 
              
                if left_max > th:
                    print('left')
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0.2
                    #self.cmd_vel_pub.publish(self.twist)
                elif rigth_max > th-0.1 and left_max<0.55:
                    print('right')
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = -0.2
                    #self.cmd_vel_pub.publish(self.twist)
                elif start_max > 0.55:
                    print('star')
                    self.flag = 40
                # else:
                
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
rospy.sleep(2)
follower = Follower()

rospy.spin()
