# Bug 2 Path Planning
[Link to Video Demo](https://www.youtube.com/watch?v=WH0H8ikGQF4&list=PLF7A_BRkte4XUmaT9YkKfNZHVgVBSWv0n&index=4&t=6s)

## Summary
This project implements the Bug 2 path planning algorithm for 6 sample worlds. The following pic explains the basic idea of it. In this project, we will move the turtlebot from a start position to a goal position along the m-line. If the turtlebot senses an obstacle, then it invokes a contour following behavior until it reaches the m-line again, at which point it proceeds towards the goal. This behavior (follow m-line, follow contour, re-acquire m-line) continues as long as there are obstacles or until the goal position is reached. 
![Bug 2 algorithm for path planning](https://github.com/YiyangQian/W4733-Robotics/blob/master/Bug2PathPlanning/bug2.png)

## Prerequisites
Set up a working Ubuntu 14.04 machine. 

## Set up environment for gazebo turtlebot
Install turtlebot_gazebo package
```
sudo apt-get install ros-indigo-turtlebot-gazebo
```

To test installation by running:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

## Run the Script
load specified world (the path to world file should be absolute address)
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/bug2_0.world
```

Then, open a new terminal at the path of lab2.py and run following
```
python bug2.py cmd_vel:=cmd_vel_mux/input/teleop
```

## Brief Description of Methods
* All methods should be self-explained.

## Explanation of logic: 
1. flag is_following_m_line will be turned into false when sensored obstacle ahead
2. once is_following_m_line turns false, the state of the robot changes into following the obstacle
    * keeps turns left when obstacle is too close, once distance to obstacle larger than 1.3, will move forward
    * turns right when robot gets too far away with obstacle
    * cases of dead loop turning right and left back and forth is avoided by using a flag rotate_flag, we force the robot not to turn right right after turning left
3. saved_pos stores all positions we have hit on m-line
4. flags have_been_here and we_are_closer check status once the robot hit m-line
    * if we are closer and we have not been this position, we turns the robot around to heading the destination
