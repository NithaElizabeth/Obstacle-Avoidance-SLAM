#!/usr/bin/env python

## @package verbal_interaction
"""This is the verbal interaction node that lets the operator communicate
with the robot

This publishes the command over the topic "verbal_command"
"""

"""Importing variables and libraries necessary for working of this node"""
import rospy
import time
import random
from std_msgs.msg import String

"""The room_list attribute contains the list of the rooms available"""

room_list	= ['entrance', 'closet', 'livingroom','kitchen', 'bathroom', 'bedroom']

"""A scaling factor"""
sim_scale = rospy.get_param('sim_scale')


def verbal_interaction():

    rospy.init_node('verbal_interaction', anonymous = True)
    rate = rospy.Rate(200)
    ## topic used to publish commands
    voice_pub = rospy.Publisher('verbal_command', String, queue_size=10)
    time.sleep(random.randint(80, 120) / sim_scale)
    while not rospy.is_shutdown():
        while (rospy.get_param('state') == 'sleep' or rospy.get_param('state') == 'find'):
            rate.sleep()
        if rospy.get_param('state') == 'normal':
            voice_pub.publish('play')
            rospy.loginfo('The Human commands to PLAY')
            while (rospy.get_param('state') != 'play'):
                rate.sleep()
        
        while (rospy.get_param('state') == 'play' or rospy.get_param('state') == 'find'):
            while (rospy.get_param('is_play_state') ==  0):
                rate.sleep()
            room_choice = random.randint(0, 5)
            rospy.loginfo('The Human commands to go to the %s', room_list[room_choice])
            voice_pub.publish(room_list[room_choice])
            
            while (rospy.get_param('is_play_state') !=  2):
                rate.sleep()
            rate.sleep()
        rospy.loginfo('The Robot stops playing')
        time.sleep(random.randint(250, 300) / sim_scale)
        rate.sleep()

if __name__ == '__main__':
    try:
        verbal_interaction()
    except rospy.ROSInterruptException:
        pass
