# Visual SLAM and Obstacle Avoidance
## Experimental Robotics Laboratory - Assignment 3
## Intoduction

This repository contains the Assignment 3 of Experimental Robotics Lab.The aim of this assignment is to implement simulation for mapping and navigation using the behavioural architecure's state machine that was completed as earlier, for a pet (dog-like) robot that moves in a (house-like)discrete 2D envirionment.
The architecture involves nodes for verbal interaction, capturing the envirionment using the camera, a finite state machine as the command manager and a service to attain the locations of different coloured balls.\
The project was developed on ROS-kinetic and Python and state machine is implemented on Smach. 

## Software Architecture

![envt](https://user-images.githubusercontent.com/47361086/109238590-3c722480-77ed-11eb-811c-12132906b905.PNG)\
The picture above is the simulated envirionment of the implemented system along with the modelled robot.As in the picture, the envirionment resembles a house-like envirionment with walls seperating the rooms.There are six different rooms and each room is depicted by a coloured ball as given below :
* Blue - Entrance
* Red - Closet
* Green - Living room
* Yellow - Kitchen
* Orange - Bathroom
* Black - Bedroom

![robot_side](https://user-images.githubusercontent.com/47361086/109238596-3ed47e80-77ed-11eb-9a86-08f6d3959f1a.PNG)\
The robot model used in this assignemnt is similar to the robot in previous assignment except that an additional laser sensor is attached to the robot chasis and also the neck joint of this robot is fixed. The pet robot has a head attached to a cylindrical neck.The head-neck setup is attached to the chasis as shown in figure.\
The major components of the system are :
* State Machine
* Verbal Interaction
* Camera Capture
* Ball
#### State Machine
This acts as the command manager.It switches between four states i.e. "sleep", "normal", "play" and "found". "Sleep" being the initial state, the robot moves to homeposition using the move_base algorithm and rest in its home location. At "normal" state, the robot wanders randomly throughout the envirionment to all the randomly generated target locations. On getting a verbal command to play, the robot will switch to the "play" state otherwise it continues it traversal and after some time it shift back to the "sleep" state.  On detecting the command, the robot changes its state to "play". In the "play" state , the robot waits for the operator to provide a location of the room it can travel to. On getting a command, the robot will check if the requested lokation is already know.If the location of the commanded room is known to the robot, then it will move to that location. Otherwise, if the room's location is unknow then the robot will switch its state to "find". In the "find" state, the robot searches for the received location using the explore-lite algorithm.
##### ROS Parameters used in state machine
* "state" - This parameter specifies the current state of the robot
* "sim_scale" - This is a scaling parameter
* "is_ball_found" - This parameter will be 1 if a new room is detected by the robot
* "is_ball_known" - This parameter cheks if the ball/room is already knwon to the robot
* "is_play_state" - To check whether the robot is currently in play state. It can have three levels i.e 0 if not in play state, 1 if the robot is at home position while being in the play state and 2 when the robot has reached any room while being in the play state.

#### The Verbal Interaction Node
This component is responsible for giving the verabal orders to the robot.This robot publishes the command over the topic "verbal_command". It is a string that would kind withe a command for play or the name of the room to which the robot is asked to navigate.
##### ROS Parameters used in Verbal Interaction
* "state" - This parameter specifies the current state of the robot
* "sim_scale" - This is a scaling parameter
* "is_play_state" - To check whether the robot is currently in play state. It can have three levels i.e 0 if not in play state, 1 if the robot is at home position while being in the play state and 2 when the robot has reached any room while being in the play state.

#### The Camera Capture Node
This component is responsible for scanning the 2-D world through which the robot travels.It will give an immediate feedback of the visuals that ideally the robot sees to the screen. This component also moves the robot to ball if the robot is in "normal" or "find" state. It also enables the robot to store the location of the newly detected coloured balls which corresponds to different rooms within the 2-D world.
##### ROS Parameters used in Camera Capture
* "state" - This parameter specifies the current state of the robot
* "is_ball_found" - This parameter will be 1 if a new room is detected by the robot
* "is_ball_known" - This parameter cheks if the ball/room is already knwon to the robot.
* "*colour_* x" and "*colour_* y"- This parameter specifies the x and y coordinates of the different balls (The value *colour* will be replaced by different colours correspondingly e.g "blue_x")
#### Ball 
There is a server which is responsible for providing the coordinates of the requested ball so as to obtain the location of the rooms.
##### ROS Parameters used in Ball
* "*colour_* x" and "*colour_* y"- This parameter specifies the x and y coordinates of the different balls (The value *colour* will be replaced by different colours correspondingly e.g "blue_x")



## State Diagram
This section explains how the states are decided. As illustrated in the state diagram below, there are three states : "sleep", "normal", "play", "find".
![State](https://user-images.githubusercontent.com/47361086/109240665-5281e400-77f1-11eb-99d2-fa374e21d28f.png)


The state "sleep" is the initial state. In the "sleep" , the robot returns to its home position and rests.Here the move_base algorith is used for the purpose of navigation through an action client.From the "sleep" state the robot switches to the "normal" behaviour after some time.In the "normal" behaviour the robot moves randomly at random location within its constrainted envirionment. In the "normal" behaviour, again the robot will be moving using the move_base algorithm.In this traversal, if the robot detects a ball then it will go near the ball and with store the location coordinates of the newly detected ball. The robot continues in movement in the world to some random positions until it receives a verbal command from its operator through a topic "verbal_command" from the verbal_interation node.  From the state "normal", it can switch to either "sleep" or "play". If the robot detects a verbal comand to play while being in the "normal" state, it switches its state to the next state, which is "play". In the "play" state, the robot initially will move towards its home lacation using the move_base algorithm and waits there until it receives a new command from its operator. If the robot does not receive a command from the operator, then the robot will stay in current state for sometime and then will switch back to the "normal" state. The operator will give a command again through the topic published by the verbal_interaction node. This command will be of string type and it will contain the name of the room to which the robot must navigate to. Now if the location of the coloured ball describeing the requested room is already available, then robot will travel to that room while being in its "play" state. If the commanded room's location is unknown to the robot, then the robot swithches the state from the "play" state to the "find" state. The detections are done with the help of a camera unit mounted on the head of the robot.In this new state called "find", the robot will search for the ball that corresponds to the room type given by the operator. On detecting the ball, the robot moves closer to the ball with help of the camera_capture node and then stores this new location.Once the ball has been located and the room has been found, the robot switches back to the play state and these processes repeats.
## Package and File List
The file tree shows the various packages in this project.

![tree](https://user-images.githubusercontent.com/47361086/109244726-a47a3800-77f8-11eb-8034-7d047e12634a.PNG)


The **docs** folder contains the documentations obtained from doxgen.The **index.html** contains the html documentation of all the scripts used in this project.The **launch folder** has the **launch file** to run the project. The folder **include** comprises all the heade files. The scripts are all contained inside the **src folder**.
## Installation and Running Procedure
Clone this github repository into the src folder of the ROS workspace
```
git clone https://github.com/NithaElizabeth/Experimental-Robtics-Lab-3
```
Next the scripts had to made executable.For that navigate to the scripts folder of this repositiory.
```
cd Experimental-Robtics-Lab-3/scripts
```
```
chmod +x state_machine.py
```
```
chmod +x ball.py
```
```
chmod +x verbal_interaction.py
```
```
chmod +x camera_capture.py
```
It can also be done from the main folder 
```
chmod +x scripts\*
```
Do the same for the launch files and src files
```
cd Experimental-Robtics-Lab-3
```
```
chmod +x src\*
chmod +x launch\*
```
After this launch the program
```
roslaunch exp_assignment3 simulation.launch
```
## Working Hypothesis 
Throughout this project, it was assumed that the robot moves in discrete 2D envirionment.It implies that the position of robot at any instant will be a point with x and y coordinates. The finite state machine was built under the hypothesis that the transition between the state will be strictly like that shown in the state diagram figure given above. It was also assumed that the robot returns to the previous states after some random time if no further tasks are given. The verbal_interaction node assumes that the operator commands will be of type string and will only say "play" or the name of the room to where the robot has to transit. It is also assumed that throughout the program that the robot will process only one operation at a time and all other operation that that point will be queued and only processed after the execution of the current task (if still in the same behaviour).
## Systems Features
The system provides a well implemented behavioural architecture. This system is capable of transiting from one state to the other. When in "normal" state , the robot can move through random targets without colliding with any immobile objects in the simulated world.The ball is capable of identifiying the voice commands and moving to prescribed locations or state. The system checks whether the location ordered by the operator is within the bound of the envirionment that was predecided.  
## Systems Limitation
The system was not realised in practical scenerio, hence most of the operations are randomised and assumed inclding the time to wait. The system is only capable of processing the voice command "play" and name of the room.The robot at times does not immediatley identify the ball within the room and it even gets stuck at some scenerios in front of the ball. The robot continous in its random motion only with a 2-D envirionment. The system adheres to the predefined working scenerios and hypothesis but it has flaws and would not be fully functional in real scenerio with lot of uncertainities.
## Possible Improvements
The system could be more rondomised so as to work in the worldly scenerio with ot of ambiguities.Rather than fixing the operator's position, it can be made random.Similarly the the envirionment can be remodelled.  Also it could be reprogrammed in such a way that the verbal_interaction node can possibly processes several voice commands rather than the one mentioned alone.Finally, the delay of the program could be more optimised.
## System Requirements
Inorder to run this program, the PC must be equipped with : 
* Python
* ROS Kinetic
* OpenCV 

## Author
The system was developed by Nitha Elizabeth John with the help from collegues and under the guidance of Prof.Luca Buoncompagni and Prof.Carmine Recchiuto\
Author  : Nitha Elizabeth John\
Contact : nithaelizabethjohn@gmail.com
