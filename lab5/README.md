# Lab5 Follow Bot
[Detailed Requirements](https://github.com/jingxixu/followbot)

## Demo
[YouTube Link](https://www.youtube.com/watch?v=kD-ap7MQoaw&list=PLF7A_BRkte4Ua944p8KgcaKsPgaj3kDZb)

## Run the script
For four parts, this command needs to be run first. 
```
roscore
```
To set up the environment of part 1, open a new terminal
```
roslaunch followbot launch.launch
```

To run the code for part 1, open another terminal
```
python follower.py
```

To set up the environment of part 2, open a new terminal
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```

To run the code for part 1, open another terminal
```
python follower_part2.py
```

To set up the environment of part 3, open a new terminal
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```

To run the code for part 1, open another terminal
```
python follower_part3.py
```

To set up the environment of part 4, open a new terminal
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=extra.world
```

To run the code for part 1, open another terminal
```
python follower_part4.py
```

### Brief Description of Methods
* All methods should be self-explained.

### Explanation of logic: 
1. follower.py is the basic version for following yellow mark lines. 
2. follower_part2.py added green, blue and red masks for detecting these colors from input image, and make movements accordingly. 
3. follower_part3.py utilized template matching supported by opencv to detect triangles and star. Templates were pre generated and stored in the template folder.
4. follower_part4.py is different from part 3 as there is no red color to tell the robot that there is a sign ahead, so template matching keeps going when follower_part4.py runs. In addition to make robot more stable, the view of the robot is limited to the center of the input image both width and height. 
