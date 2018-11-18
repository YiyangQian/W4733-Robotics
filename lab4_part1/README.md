# Lab4 part1 
[Detailed Requirements](http://www.cs.columbia.edu/~allen/F18/HMWK/lab4/lab4.html)

## Run the script 
```
python draw.py obstacle.txt start_end_points.txt
```


### Brief Description of Methods
* All methods should be self-explained.

### Explanation of logic: 
1. We use the basic draw.py as the start point of our project, since it draws the basic environment for us. 
2. A class named RRT was defined in rrt.py, this is the core part of our project. An RRT is designed as following: 
    * An instance of RRT is offered with start and ends points, step size and a list of all vertices. 
    * Each step we generate a rondom point(5% of target as bias) called goNode and then find the current closest existing point, and generate a node called nextPotentailNode. 
    * Then we valid the nextPotentailNode from two aspects. First, this potential node cannot cannot generate a path cross any obstacle edges. Second, this potentail cannot be too close to any existing node, and we took two node too close to each other(within one step) as duplicate, and the potentail node is not valid. 
    * Finally we check the terminal situtation. The final situation is defined as: 1) distance to target within one step, and the potential path between this node and target will not cross any obstacle edges.  

## Examples of resultes
![step size 10](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab4_part1/step_10.png)

![step size 25](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab4_part1/step_25.png)

![step size 50](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab4_part1/step_50.png)