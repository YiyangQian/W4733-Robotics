# Lab 2
[Link to Demo](https://www.youtube.com/watch?v=WH0H8ikGQF4&list=PLF7A_BRkte4XUmaT9YkKfNZHVgVBSWv0n&index=4&t=6s)

[Detailed Requirements](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab2/lab2.pdf)

### Run the Script
load specified world (the path to world file should be absolute address)
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/bug2_0.world
```

Then, open a new terminal and run the python program
```
python lab2.py cmd_vel:=cmd_vel_mux/input/teleop
```

### Brief Description of Methods
* All methods should be self-explained.

### Explanation of logic: 
1. flag is_following_m_line will be turned into false when sensored obstacle ahead
2. once is_following_m_line turns false, the state of the robot changes into following the obstacle
    * keeps turns left when obstacle is too close, once distance to obstacle larger than 1.3, will move forward
    * turns right when robot gets too far away with obstacle
    * cases of dead loop turning right and left back and forth is avoided by using a flag rotate_flag, we force the robot not to turn right right after turning left
3. saved_pos stores all positions we have hit on m-line
4. flags have_been_here and we_are_closer check status once the robot hit m-line
    * if we are closer and we have not been this position, we turns the robot around to heading the destination