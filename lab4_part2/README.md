# Lab4 part2
[Detailed Requirements](http://www.cs.columbia.edu/~allen/F18/HMWK/lab4/lab4.html)

## Run the script 
```
python draw.py obstacle.txt start_end_points.txt
```

### Brief Description of Methods
* All methods should be self-explained.

### Explanation of logic: 
1. Lab4_part was made on top the Lab4_part1 and still use the basic draw.py as the start point of our project.
2. A class named BiDirectionRRT was defined in BiDirectionRRT.py. An BiDirectionRRT is designed as following: 
    * An instance of BiDirectionRRT will make two RRT instances inside, and each of them have seperate step sizes. 
    * An instance of BiDirectionRRT is offered with start and ends points, step sizes and a list of all vertices. 
    * Two instances of RRT take turn to generate a random direction and try to move. 
    * The stop case is checked by the mothod called is intersected. It takes the latest generated point and compares this point with all points in the other RRT instance, if one has distance within one step and there is no obstacle between them, then we are sure that we have already found a path. 

## Examples of resultes
![step size 10 and 25](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab4_part2/step_10_25.png)

![step size 25 and 10](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab4_part2/step_25_10.png)

![step size 25 and 25](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab4_part2/step_25_25.png)

![step size 25 and 50](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab4_part2/step_25_50.png)