#!/usr/bin/env python

## @package state_machine
"""This is the state machine that switches between the four main states

This node is a satate machine that is implemented in position of command
manager in the whole architecture.This state machine is implemented in 
smach.It switches between four states sleep, norma, play and find. 
"""

"""Importing variables and libraries necessary for working of this node"""
import rospy
import smach
import smach_ros
import time
import random
import actionlib
from std_msgs.msg import String
from exp_assignment3.srv import BallLocation, Explore
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

"""The parameters x_lim_max and y_lim_max  corresponds to the maximum 
boundary limit of the descrete 2D envirionment of the robot."""
x_lim_max = 9
y_lim_max = 9

"""The parameters x_lim_min and y_lim_min  corresponds to the minimum 
boundary limit of the descrete 2D envirionment of the robot."""
x_lim_min = -9
y_lim_min = -9

"""The parameters home_position_x and home_position_y has the location 
of the home as given"""

home_position_x = -5
home_position_y = 8

"""The room_list attribute contains the list of the rooms available"""

room_list	= ['entrance', 'closet', 'livingroom','kitchen', 'bathroom', 'bedroom']

"""A scaling factor"""
sim_scale = rospy.get_param('sim_scale')

"""Initialisation of variables used in this node"""
itr = 1
is_play_state = 0
room_type = 100
counter_time = random.randint(2, 7)



## define state Sleep
class Sleep(smach.State):
    """This class defines the state Sleep
	
    This is the state in which robots return to home position and does 
    not indulge in any activities. The robot navigates using the move_base 
    algorithm.
    An action client is used to reach the corresponding coordinates
    
    Methods
    ----------
    _init_(self)
		This methods initialises all the attributes of this class
    execute(self,userdata)
		This methods executes the sleep state in which the robot moves to its home position from its current location
		
	Attributes
	----------
	home_position
		A  variable of MoveBaseGoal() to store the target position
	itr
		To keep the track of iterations
	move_base_sleep
		A move_base client to send the home location
    """

    
    def __init__(self):
        
        smach.State.__init__(self, 
                             outcomes=['goto_NORMAL_state'])

    def execute(self, userdata):

        global itr
        rospy.set_param('state','sleep')
        rospy.loginfo('_______________________Executing state SLEEP ________________________')
        
        move_base_sleep = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if itr == 0:
            home_position = MoveBaseGoal()
            
            home_position.target_pose.header.frame_id = "map"
            home_position.target_pose.header.stamp = rospy.Time.now()
            home_position.target_pose.pose.position.x = home_position_x
            home_position.target_pose.pose.position.y = home_position_y
            home_position.target_pose.pose.orientation.w = 1.0
            move_base_sleep.send_goal(home_position)
            #Value 3 corresponds to SUCCESS
            while(move_base_sleep.get_state() != 3):
				rospy.Rate(200).sleep
            rospy.loginfo('The Robot has reached HOME position')
        else:
            move_base_sleep.wait_for_server() 
            itr = 0
        #Waits for some time
        time.sleep(random.randint(5, 10) / sim_scale) 
        counter_time = random.randint(2, 7)
        
        return 'goto_NORMAL_state'




class Normal(smach.State):
    
    """This class defines the state Normal
	
    This is the state in which robots moves randomly in the house using 
    move_base and is willing to listen to its operators commands. Hence 
    its subscribes to the verbal_interaction node.
    On detecting a ball, the robot stores the position of the ball 
    and correspondingly to the room.
    The robot changes the state to PLAY on the verbal command or it 
    changes the state to SLEEP randomly after some time
    
    Methods
    ----------
    _init_(self)
		This methods initialises all the attributes of this class
    execute(self,userdata)
		This methods executes the normal state in which the robot moves
		randomly until it gets a play command
	normal_callback(self,data)
		This is the normal state's call back function.It acknoledge the 
		play command
		
	Attributes
	----------
	home_position
		A  variable of MoveBaseGoal() to store the target position
	move_base_normal
		A  move_base client to send the goal location
    """

    
    
    #Initialisation
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['goto_SLEEP_state','goto_PLAY_state'])
        
        rospy.Subscriber('verbal_command', String, self.normal_callback)
    
    def execute(self, userdata):
        
        rospy.loginfo('______________________Executing state NORMAL _______________________')
        global is_play_state
        global counter_time
        rospy.set_param('state', 'normal')
        move_base_normal = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        new_goal = MoveBaseGoal()
        new_goal.target_pose.header.frame_id = "map"
        new_goal.target_pose.pose.orientation.w = 1.0
        while (is_play_state == 0 and (not rospy.is_shutdown()) and rospy.get_param('state') == 'normal'):
            is_room_found = 0
            new_goal.target_pose.header.stamp = rospy.Time.now()
            new_goal.target_pose.pose.position.x = random.randint(x_lim_min, x_lim_max)
            new_goal.target_pose.pose.position.y = random.randint(y_lim_min, y_lim_max)
            move_base_normal.send_goal(new_goal)
            print("The robot moves to a location [%d, %d].\n" %(new_goal.target_pose.pose.position.x, new_goal.target_pose.pose.position.y))
            
            while (move_base_normal.get_state() != 3 and is_room_found != 1):
                if(rospy.get_param('is_ball_found') == 1):
                    
                    print("The robot found a new room.\n")
                    while(rospy.get_param('is_ball_found') == 1):
                        is_room_found = 1
                        rospy.Rate(200).sleep
                elif (move_base_normal.get_state() > 3):
                    break
                if(is_play_state == 1):
                    break
                rospy.Rate(200).sleep
            if(move_base_normal.get_state() == 3):
                print("The Robot reached the target")
            move_base_normal.cancel_all_goals()
            counter_time = counter_time - 1
            rospy.Rate(200).sleep

        if counter_time == 0:
            print("The robot returns to SLEEP state")
            return 'goto_SLEEP_state'

        elif is_play_state == 1:
            is_play_state = 0
            return 'goto_PLAY_state'

    ##
    def normal_callback(self, data):
        global is_play_state
        if (rospy.get_param('state') == 'normal'):
            rospy.loginfo('The Robot has recieved a play command')
            move_base_normal = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            move_base_normal.cancel_all_goals()
            is_play_state = 1



## Play state
class Play(smach.State):
    
    """This class defines the state Play
	
    This is the state in which robots return to home position initially 
    and waits for the roomlocation. On attaining a command it checks with
    a service to find the coordinates.if the ball location is available 
    already then robot moves to that location otherwise switches its 
    state to Find
    The state subscribes to verbal_interaction node to get commands
    
    Methods
    ----------
    _init_(self)
		This methods initialises all the attributes of this class
    execute(self,userdata)
		This methods executes the play state in which the robot moves to
		its home position from its current location
	play_callback(self,data)
		This callback function finds the room correspondig to the ball
		
	Attributes
	----------
	home_position
		A  variable of MoveBaseGoal() to store the target position
	itr
		To keep the track of iterations
	move_base_sleep
		A move_base client to send the home location
    """

    
    
    def __init__(self):
        # initialisation function
        smach.State.__init__(self, 
                             outcomes=['goto_NORMAL_state','goto_FIND_state'])
                            
        rospy.Subscriber('verbal_command', String, self.play_callback)


    def execute(self, userdata):
		
		
	rospy.loginfo('______________________Executing state PLAY _______________________')
        global room_type, counter_time
        rospy.set_param('state', 'play')
        move_base_play = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        target_pos = MoveBaseGoal()
        target_pos.target_pose.header.frame_id = "map"
        target_pos.target_pose.header.stamp = rospy.Time.now()
        target_pos.target_pose.pose.orientation.w = 1.0
        target_pos.target_pose.pose.position.x = home_position_x
        target_pos.target_pose.pose.position.y = home_position_y
        target_pos.target_pose.pose.orientation.w = 1.0
        rospy.set_param('is_play_state', 0)
        move_base_play.send_goal(target_pos)
        while(move_base_play.get_state() != 3):
            rospy.Rate(200).sleep
        rospy.loginfo('The Robot reached the HOME position')
        rospy.set_param('is_play_state', 1)
        rospy.wait_for_message('verbal_command', String)
        while ((not rospy.is_shutdown()) and counter_time != 1 and rospy.get_param('state') == 'play'):
            rospy.set_param('is_play_state', 0)
            while(room_type == 100):
                rospy.Rate(200).sleep
            rospy.wait_for_service('BallLocation')
            ball_service_client = rospy.ServiceProxy('BallLocation', BallLocation)
            ball_location = ball_service_client(room_type)
            idx = room_type
            room_type = 100
            if (ball_location.loc.x != 100):
                rospy.loginfo('The Robot moves to the %s', room_list[idx])
                
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                target_pos.target_pose.pose.position.x = ball_location.loc.x
                target_pos.target_pose.pose.position.y = ball_location.loc.y
                target_pos.target_pose.pose.orientation.w = 1.0
                move_base_play.send_goal(target_pos)
                time.sleep(5)
                rospy.loginfo('The Robot moves to the room')
                target_pos.target_pose.header.frame_id = "map"
                target_pos.target_pose.header.stamp = rospy.Time.now()
                target_pos.target_pose.pose.position.x = home_position_x
                target_pos.target_pose.pose.orientation.w = 1.0
                target_pos.target_pose.pose.position.y = home_position_y
                move_base_play.send_goal(target_pos)
                while(move_base_play.get_state() != 3):
                    rospy.Rate(200).sleep
                rospy.loginfo('The Robot reaches HOME position')
                rospy.set_param('is_play_state', 2)
                counter_time = counter_time - 1
            else:
                rospy.set_param('is_ball_known', idx)
                break
            rospy.Rate(200).sleep

        if counter_time == 1:
            rospy.loginfo('The Robot switches back to NORMAL state')
            return 'goto_NORMAL_state'

        elif rospy.get_param('is_ball_known') != 100:
            rospy.loginfo('The robot could not find %s is. ', room_list[rospy.get_param('is_ball_known')])
            rospy.loginfo('The Robot switches to FIND state')
            return 'goto_FIND_state'

    ## Play state callback
    def play_callback(self, data):
        global room_type
        if (rospy.get_param('state') == 'play' and data.data != 'play'):
            rospy.loginfo('The robot moves to the %s', data.data)
            if data.data == room_list[0]:
                room_type = 0
            elif data.data == room_list[1]:
                room_type = 1
            elif data.data == room_list[2]:
                room_type = 2
            elif data.data == room_list[3]:
                room_type = 3
            elif data.data == room_list[4]:
                room_type = 4
            else:
                room_type = 5



## Find state
class Find(smach.State):
    
    """This class defines the state Find
	
    This is the state in which robots searches for the ball asked in the
    Play state.This state uses explorer_lite algorithm. On matching the 
    ball this state swtches back to Play state.
    The state subscribes to verbal_interaction node to get commands
    
    Methods
    ----------
    _init_(self)
		This methods initialises all the attributes of this class
    execute(self,userdata)
		This methods executes the play state in which the robot moves to
		its home position from its current location
	play_callback(self,data)
		This callback function finds the room correspondig to the ball
		
	Attributes
	----------
	home_position
		A  variable of MoveBaseGoal() to store the target position
	itr
		To keep the track of iterations
	explore_start
		A Explorer Flag
    """

    
    
    ## Find state initialization: set the outcomes
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['goto_PLAY_state'])


    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        rospy.loginfo('______________________Executing state FIND _______________________')
        rospy.set_param('state', 'find')

        rospy.wait_for_service('start_flag')
        explore_start = rospy.ServiceProxy('start_flag', Explore)
        explore_start(1)
        rospy.loginfo('The Robot is searching for the Room')
        while ((not rospy.is_shutdown()) and rospy.get_param('state') == 'find' and rospy.get_param('is_ball_known') != 100):
            rospy.Rate(200).sleep
        rospy.loginfo('The Robot found the room')
        rospy.set_param('is_play_state', 2)
        rospy.set_param('is_ball_known', 100)
        return 'goto_PLAY_state'




# SMACH state machine 
def main():
    """This class is the main class
	
    This class defines the relation of each state parameters to corresponding 
    states.From the sleep state , the robot moves to Normal state.From the 
    normal state the robot can move to either Sleep state or PLay state.And 
    from the Play state the robot moves back to normal state or Find state.
    From the Find state the robot shifts back to the play state
		
    """	

    rospy.init_node('state_machine', anonymous = True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'goto_NORMAL_state':'NORMAL'})
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'goto_PLAY_state':'PLAY', 
                                            'goto_SLEEP_state':'SLEEP'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'goto_NORMAL_state':'NORMAL',
                                            'goto_FIND_state':'FIND'})
        smach.StateMachine.add('FIND', Find(), 
                               transitions={'goto_PLAY_state':'PLAY'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # outcome = sm.execute() # an output variable is to be used if
                             # this finite state machine is nested
                             # inside another one

    # Execute the state machine
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
