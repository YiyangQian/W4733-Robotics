# Lab 3
[Link to Demo](https://www.youtube.com/watch?v=k-jknoDQJ8o)

[Detailed Requirements](https://github.com/jingxixu/vgraph)

### Run the Script
load specified world 
```
roslaunch vgraph launch.launch
```

Then, open a new terminal at the path of p4.py and p5.py and run following
```
python p4.py
python p5.py
```

### Brief Description of Methods
* All methods should be self-explained.

### Explanation of logic: 
1. The logic of generating the convex hull is in convex_hull.py.
2. The logic for generating all possible edges is in p4.py, and once an edge is determined as valid will be published.
3. We ran floyd algorithm for finding the shortest path. 
4. To move to robot to the destination, we pass an array of path to p5.py, and move the robot by running p5.py.