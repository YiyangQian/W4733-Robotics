# Labe 1 
[Detailed Requirements](https://github.com/YiyangQian/W4733-Robotics/blob/master/lab1/lab1.pdf)

### Run the Script
Set up a turtlebot environment
```
roslaunch rbx1_bringup fake_turtlebot.launch
```

Then, open a new terminal and start an rviz simulator
```
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```

Open another terminal and run the script
```
python timed_out_and_back.py
```

Then prompt will ask for input(T/R/Q). T stands for transfer, R stands for rotation, and Q stands for quit. After entering T or R, terminal will ask for a number, which stands for the goal distance/angular. 
