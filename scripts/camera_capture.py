#!/usr/bin/env python

## @package camera_capture
"""This helps in attaining the visual details of the envirionment with 
the use ofOpenCV

This node contantly scans the envirionment and also detects the coloured
ball that corresponds to different rooms. On finding a new ball it helps 
the robot to move near to that ball 
"""

"""Importing variables and libraries necessary for working of this node"""

import sys
import numpy as np
import imutils
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import actionlib
from exp_assignment3.srv import Explore
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction

"""The lower bound of the RGB color coding for different balls"""

blueLower = (100, 50, 50)
yellowLower = (25, 50, 50)
redLower = (0, 50, 50)
magentaLower = (125, 50, 50)
greenLower = (50, 50, 50)
blackLower = (0, 0, 0)

"""The upper bound of the RGB color coding for different balls"""

blueUpper = (130, 255, 255)
yellowUpper = (35, 255, 255)
redUpper = (5, 255, 255)
magentaUpper = (150, 255, 255)
greenUpper = (70, 255, 255)
blackUpper = (5, 50, 50)

"""Initialisation of variables used in this node"""

is_blue = 0
is_red = 0
is_green = 0
is_yellow = 0
is_magenta = 0
is_black = 0

VERBOSE = False

"""This class performs the scanning and image processing"""
class image_feature:

    
    def __init__(self):
	"""Initialize ros publisher, ros subscriber"""
        rospy.init_node('camera_capture', anonymous = True)

        ## topic wgere we publish published
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        ## topic where we robot velocity commands
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        ## subscribed topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    ##for contours
    def contours(self, hsv, colorLower, colorUpper):
            colorMask = cv2.inRange(hsv, colorLower, colorUpper)
            colorMask = cv2.erode(colorMask, None, iterations=2)
            colorMask = cv2.dilate(colorMask, None, iterations=2)
            colorCnts = cv2.findContours(colorMask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            colorCnts = imutils.grab_contours(colorCnts)
            return colorCnts


    def callback(self, ros_data):
	"""Callback function of subscribed topic"""
        global is_blue, is_red, is_green,is_yellow, is_magenta, is_black

        ##direct conversion to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:



        if (rospy.get_param('state') == 'normal'):

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            blueCnts = image_feature.contours(self, hsv, blueLower, blueUpper)
            redCnts = image_feature.contours(self, hsv, redLower, redUpper)
            greenCnts = image_feature.contours(self, hsv, greenLower, greenUpper)
            yellowCnts = image_feature.contours(self, hsv, yellowLower, yellowUpper)
            magentaCnts = image_feature.contours(self, hsv, magentaLower, magentaUpper)
            blackCnts = image_feature.contours(self, hsv, blackLower, blackUpper)

            center = None
            # only proceed if at least one contour was found

            if(len(blueCnts) > 0 and is_blue != 2 \
                     and is_red != 1 and is_green != 1 and is_yellow != 1 \
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## action client used to stop move_base execution
                move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                move_base_client.cancel_all_goals()
                is_blue = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # draw a red dot
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of ENTRANCE
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('blue_x', pos.pose.pose.position.x)
                    rospy.set_param('blue_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the ENTRANCE is stored')
                    is_blue = 2
                    rospy.set_param('i', 0)

            elif(len(redCnts) > 0 and is_red != 2 \
                     and is_blue != 1 and is_green != 1 and is_yellow != 1 \
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## action client used stop move_base execution
                move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                move_base_client.cancel_all_goals()
                is_red = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of CLOSET
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('red_x', pos.pose.pose.position.x)
                    rospy.set_param('red_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the CLOSET is stored')
                    is_red = 2
                    rospy.set_param('is_ball_found', 0)

            elif(len(greenCnts) > 0 and is_green != 2 \
                     and is_blue != 1 and is_red != 1 and is_yellow != 1 \
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## action client to stop move_base execution
                move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                move_base_client.cancel_all_goals()
                is_green = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of LIVING ROOM
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('green/x', pos.pose.pose.position.x)
                    rospy.set_param('green/y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the LIVING ROOM is stored')
                    is_green = 2
                    rospy.set_param('is_ball_found', 0)

            elif(len(yellowCnts) > 0 and is_yellow != 2 \
                     and is_blue != 1 and is_red != 1 and is_green != 1
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## action client used stop move_base execution
                move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                move_base_client.cancel_all_goals()
                is_yellow = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of KITCHEN
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('yellow_x', pos.pose.pose.position.x)
                    rospy.set_param('yellow_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the KITCHEN is stored')
                    is_yellow = 2
                    rospy.set_param('is_ball_found', 0)

            elif(len(magentaCnts) > 0 and is_magenta != 2 \
                     and is_blue != 1 and is_red != 1 and is_green != 1
                     and is_yellow != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## action client to stop move_base execution
                move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                move_base_client.cancel_all_goals()
                is_magenta = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of BATHROOM
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('magenta_x', pos.pose.pose.position.x)
                    rospy.set_param('magenta_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the BATHROOM is stored')
                    is_magenta = 2
                    rospy.set_param('is_ball_found', 0)

            elif(len(blackCnts) > 0 and is_black != 2 \
                     and is_blue != 1 and is_red != 1 and is_green != 1
                     and is_yellow != 1 and is_magenta != 1):
                rospy.set_param('is_ball_found', 1)
                ## action client used to stop move_base execution
                move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                move_base_client.cancel_all_goals()
                is_black = 1
                # find the largest contour in the  mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of BEDROOM
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('black_x', pos.pose.pose.position.x)
                    rospy.set_param('black_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the BEDROOM is stored')
                    is_black = 2
                    rospy.set_param('is_ball_found', 0)



        elif(rospy.get_param('state') == 'find'):

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            blueCnts = image_feature.contours(self, hsv, blueLower, blueUpper)
            redCnts = image_feature.contours(self, hsv, redLower, redUpper)
            greenCnts = image_feature.contours(self, hsv, greenLower, greenUpper)
            yellowCnts = image_feature.contours(self, hsv, yellowLower, yellowUpper)
            magentaCnts = image_feature.contours(self, hsv, magentaLower, magentaUpper)
            blackCnts = image_feature.contours(self, hsv, blackLower, blackUpper)

            center = None
            # only proceed if at least one contour was found

            if(len(blueCnts) > 0 and is_blue != 2 \
                     and is_red != 1 and is_green != 1 and is_yellow != 1
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## explore lite service client used to
                # stop explore_lite algorithm execution
                rospy.wait_for_service('stop_flag')
                explore_stop = rospy.ServiceProxy('stop_flag', Explore)
                explore_stop(0)
                is_blue = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blueCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of ENTRANCE
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('blue_x', pos.pose.pose.position.x)
                    rospy.set_param('blue_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the ENTRANCE is stored')
                    is_blue = 2
                    rospy.set_param('is_ball_found', 0)
                    if rospy.get_param('is_ball_known') == 0:
                        
                        rospy.set_param('is_ball_known', 100)
                    else:
                        ## explore_lite starting
                        rospy.wait_for_service('start_flag')
                        explore_start = rospy.ServiceProxy('start_flag', Explore)
                        explore_start(1)

            elif(len(redCnts) > 0 and is_red != 2 \
                     and is_blue != 1 and is_green != 1 and is_yellow != 1
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## explore lite starting
                rospy.wait_for_service('stop_flag')
                explore_stop = rospy.ServiceProxy('stop_flag', Explore)
                explore_stop(0)
                is_red = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(redCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of CLOSET
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('red_x', pos.pose.pose.position.x)
                    rospy.set_param('red_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the CLOSET is stored')
                    is_red = 2
                    rospy.set_param('is_ball_found', 0)
                    if rospy.get_param('is_ball_known') == 1:
                        rospy.set_param('is_ball_known', 100)
                    else:
                        ## explore_lite starting
                        rospy.wait_for_service('start_flag')
                        explore_start = rospy.ServiceProxy('start_flag', Explore)
                        explore_start(1)

            elif(len(greenCnts) > 0 and is_green != 2 \
                     and is_blue != 1 and is_red != 1 and is_yellow != 1 \
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## explore lite stoping
                rospy.wait_for_service('stop_flag')
                explore_stop = rospy.ServiceProxy('stop_flag', Explore)
                explore_stop(0)
                is_green = 1
                # find the largest contour in the green mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(greenCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of LIVINGROOM
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('green_x', pos.pose.pose.position.x)
                    rospy.set_param('green_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the LIVINGROOM is stored')
                    is_green = 2
                    rospy.set_param('is_ball_found', 0)
                    if rospy.get_param('is_ball_known') == 2:
                        rospy.set_param('is_ball_known', 100)
                    else:
                        ## explore_lite starting
                        rospy.wait_for_service('start_flag')
                        explore_start = rospy.ServiceProxy('start_flag', Explore)
                        explore_start(1)

            elif(len(yellowCnts) > 0 and is_yellow != 2 \
                     and is_blue != 1 and is_red != 1 and is_green != 1 \
                     and is_magenta != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## explore lite stoping
                rospy.wait_for_service('stop_flag')
                explore_stop = rospy.ServiceProxy('stop_flag', Explore)
                explore_stop(0)
                is_yellow = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(yellowCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of KITCHEN
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('yellow_x', pos.pose.pose.position.x)
                    rospy.set_param('yellow_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the KITCHEN is stored')
                    is_yellow = 2
                    rospy.set_param('is_ball_found', 0)
                    if rospy.get_param('is_ball_known') == 3:
                        rospy.set_param('is_ball_known', 100)
                    else:
                        ## explore_lite starting
                        rospy.wait_for_service('start_flag')
                        explore_start = rospy.ServiceProxy('start_flag', Explore)
                        explore_start(1)

            elif(len(magentaCnts) > 0 and is_magenta != 2 \
                     and is_blue != 1 and is_red != 1 and is_green != 1 \
                     and is_yellow != 1 and is_black != 1):
                rospy.set_param('is_ball_found', 1)
                ## explore lite stopping
                rospy.wait_for_service('stop_flag')
                explore_stop = rospy.ServiceProxy('stop_flag', Explore)
                explore_stop(0)
                is_magenta = 1
                # find the largest contour in the  mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(magentaCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 255, 0), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of BATHROOM
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('magenta_x', pos.pose.pose.position.x)
                    rospy.set_param('magenta_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the BATHROOM is stored')
                    is_magenta = 2
                    rospy.set_param('is_ball_found', 0)
                    if rospy.get_param('is_ball_known') == 4:
                        rospy.set_param('is_ball_known', 100)
                    else:
                        ## explore_lite starting
                        rospy.wait_for_service('start_flag')
                        explore_start = rospy.ServiceProxy('start_flag', Explore)
                        explore_start(1)

            elif(len(blackCnts) > 0 and is_black != 2 \
                     and is_blue != 1 and is_red != 1 and is_green != 1 \
                     and is_yellow != 1 and is_magenta != 1):
                rospy.set_param('is_ball_found', 1)
                ## explore lite stopping
                rospy.wait_for_service('stop_flag')
                explore_stop = rospy.ServiceProxy('stop_flag', Explore)
                explore_stop(0)
                is_black = 1
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(blackCnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if(center[0] < 396 or center[0] > 404 or radius < 99 or radius > 101):
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002 * (center[0] - 400)
                    vel.linear.x = -0.01 * (radius - 100)
                    if(vel.linear.x > 0.8):
                        vel.linear.x = 0.8
                    self.vel_pub.publish(vel)
                else:
                    # storing the coordinates of BEDROOM
                    pos = rospy.wait_for_message('odom', Odometry, timeout = None)
                    rospy.set_param('black_x', pos.pose.pose.position.x)
                    rospy.set_param('black_y', pos.pose.pose.position.y)
                    rospy.loginfo('Location of the BEDROOM is stored')
                    is_black = 2
                    rospy.set_param('is_ball_found', 0)
                    if rospy.get_param('is_ball_known') == 5:
                        rospy.set_param('is_ball_known', 100)
                    else:
                        ## explore_lite starting
                        rospy.wait_for_service('start_flag')
                        explore_start = rospy.ServiceProxy('start_flag', Explore)
                        explore_start(1)



        cv2.imshow('window', image_np)
        cv2.waitKey(2)


def main(args):
    """This class is the main class
	
    The execution starts at the main function
    """	

    #Initializes and cleanups ros node
    #ic = image_feature()
    image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
