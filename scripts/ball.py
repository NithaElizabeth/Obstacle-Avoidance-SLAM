#!/usr/bin/env python

## @package ball
"""This is the ball service server

This provides the coordinates of the requested ball through a parameter 
that corresponds to the x and y locations. 
"""

"""Importing variables and libraries necessary for working of this node"""
import rospy
from exp_assignment3.srv import BallLocation
from exp_assignment3.msg import Coordinates

## BallLocation service callback
def check_ball_room(req):
    loc = Coordinates()
    if req == 0:
        loc.x = rospy.get_param('blue_x')
        loc.y = rospy.get_param('blue_y')
    elif req == 1:
        loc.x = rospy.get_param('red_x')
        loc.y = rospy.get_param('red_y')
    elif req == 2:
        loc.x = rospy.get_param('green_x')
        loc.y = rospy.get_param('green_y')
    elif req == 3:
        loc.x = rospy.get_param('yellow_x')
        loc.y = rospy.get_param('yellow_y')
    elif req == 4:
        loc.x = rospy.get_param('magenta_x')
        loc.y = rospy.get_param('magenta_y')
    else:
        loc.x = rospy.get_param('black_x')
        loc.y = rospy.get_param('black_y')
    return loc

## Initialization
def ball_server():
    rospy.init_node('ball')
    s = rospy.Service('BallLocation', BallLocation, check_ball_room)
    rospy.spin()

if __name__ == "__main__":
    ball_server()
