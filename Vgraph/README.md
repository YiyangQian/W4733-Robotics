# Visibility Graph Path Planning
[Link to Demo](https://www.youtube.com/watch?v=k-jknoDQJ8o)

## Summary
This project implements [visibility graph (vgraph)](https://en.wikipedia.org/wiki/Visibility_graph) path planning algorithm using the ArbotiX turtlebot simulator and visualize the graph and robot path following in RViz.

## Prerequisites
* python 2.7
* ROS Indigo
* Ubuntu 14.04 with OpenCV 3.1.0 and numpy 1.15.1
* ros-by-example code

## Files
`data/world_obstacles.txt`: Text file indicating obstacle locations (in **cm**).
  - First line is the total number of obstacles in the world.
  - Each obstacle starts with a number specifying total number of vertices, followed by these vertices' location in counter-clockwise order.
- `data/goal.txt`: Location of the goal (in **cm**).
- `src/create_map.py`: Creates or updates the `map.png` under `maps/` folder, using obatacles and goal defined in the above files.
- `launch/launch.launch`: Launch file to start everything you need.
- `maps/map.png`: Image of the map.
- `maps/map.yaml`: Map metadata.
- `package.xml`: Package metadata.
- `world.rviz`: RViz configuration file.

## Map
The map we use in RViz is 1200cm by 600cm, top left and bottom right are (-300, 300) and (900, -300) respectively. Also note that cells in the RViz grid are 50cm by 50cm. The start position of the robot is always at (0, 0), facing x-positive and the goal is defined in `data/goal.txt`. The obstacles are orange polygons and the goal is a purple dot. More about using the `map_server` package [here](http://wiki.ros.org/map_server).

## Run
load specified world 
```
roslaunch vgraph launch.launch
```

Then, open a new terminal at the path of plan.py and move.py and run following
```
python plan.py
python move.py
```

### Brief Description of Methods
* All methods should be self-explained.

### Explanation of logic: 
1. convex_hull.py contains the logic of generating the convex hull.
2. The logic for generating all possible edges is in plan.py, and once an edge is determined as valid will be published.
3. Floyd algorithm for finding the shortest path is applied in plan.py. 
4. To move to robot to the destination, we pass an array of path to move.py, and move the robot by running move.py.

## Steps
1. Bring up ArbotiX turtlebot simulator and RViz with the map of obstacles and goal.
<p align="center">
  <img src="imgs/map.png">
</p>

2. Grow the obstacles using reflection algorithm (flip the robot around the reference point, place it at each vertex of the obstacle, and create a convex hull). We assume the robot to be a 36cm-by-36cm square. 
<p align="center">
  <img src="imgs/grown.png">
</p>

3. Create the vgraph by first fully connecting all obstacle vertices + start + goal and then implement a collision checker to remove edges that collides with obstacles (except endpoints).
<p align="center">
  <img src="imgs/graph.png">
</p>

4. Implement Floyd algorithm for finding a shortest path from start to goal. 
<p align="center">
  <img src="imgs/path.png">
</p>

5. Move the robot following the shortest path.
<p align="center">
  <img src="imgs/demo.gif">
</p>