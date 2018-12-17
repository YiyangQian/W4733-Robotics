# Bi-directional Rapidly-exploring Random Tree (RRT) Path Planning

## Summary
This project is based on the RRT project which is sibling to this project. To optimize the path planning process, a bi-directional RRT builds two trees starting from start point and end point at the same time. The two trees take turns to generate a new node, and check whether any nodes in the other tree is close enough to the new node. If there is any pair from two trees get close enough and there is no obstacle between them, we know we have already found a path.

## Run the script 
```
python draw.py obstacle.txt start_end_points.txt
```

## Brief Description of Methods
All methods should be self-explained.

## Explanation of logic: 
1. Lab4_part was made on top the Lab4_part1 and still use the basic draw.py as the start point of our project.
2. A class named BiDirectionRRT was defined in BiDirectionRRT.py. An BiDirectionRRT is designed as following: 
    * An instance of BiDirectionRRT will make two RRT instances inside, and each of them have seperate step sizes. 
    * An instance of BiDirectionRRT is offered with start and ends points, step sizes and a list of all vertices. 
    * Two instances of RRT take turn to generate a random direction and try to move. 
    * The stop case is checked by the mothod called is intersected. It takes the latest generated point and compares this point with all points in the other RRT instance, if one has distance within one step and there is no obstacle between them, then we are sure that we have already found a path. 

## Examples of resultes
step size 10 and 25
![step size 10 and 25](https://github.com/YiyangQian/W4733-Robotics/blob/master/BiDirectionalRRT/step_10_25.png)

step size 25 and 10
![step size 25 and 10](https://github.com/YiyangQian/W4733-Robotics/blob/master/BiDirectionalRRT/step_25_10.png)

step size 25 and 25
![step size 25 and 25](https://github.com/YiyangQian/W4733-Robotics/blob/master/BiDirectionalRRT/step_25_25.png)

step size 25 and 50
![step size 25 and 50](https://github.com/YiyangQian/W4733-Robotics/blob/master/BiDirectionalRRT/step_25_50.png)