# Translation and Rotation
[Link to Video Demo](https://youtu.be/ydDKaR5_Vos)

## Summary
This project is an interface of an virtual ROS indigo robot. The robot reads in commands from command line: T for translation, R for rotation and Q for quit. In addition, if T (translation) or R (rotation), it will prompt for the distance to move or angles to rotate. The project support positive and negative translations (forward and backward movement) and positive and negative rotations (counterclockwise and clockwise rotation). It will keeps prompting until user hits Q. 

## Prerequest
1. Set up a working Ubuntu 14.04 machine. 
2. Install ROS Indigo. 

## Run the Script
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
python runIndigo.py
```

Then prompt will ask for a command for the next move, and you can type in T(Transfer), R(Rotate) or Q(Quit). 

If the move ordered is T or R, a number will be asked for. The number you type in means target distance to move or target angular to rotate.

## Brief Description of Methods
* def stop(self):  
    * stop the robort after one action, called as the last step of transfer or rotate
* def translate(self, linear_speed, linear_duration, rate, r): 
    * translate the robort accroding to params passed in.
* def rotate(self, angular_speed, angular_duration, rate, r): 
    * rotate the robort accroding to params passed in.
* def getTParams(self, linear_speed): 
    * ask for a goal distance as input from user, and generate params needed to tranfer robort.
* def getRParams(self, angular_speed): 
    * ask for a goal angular as input from user, and generate params needed to rotate robort.

### Generate tar.gz file
```
tar -zcvf file.tar.gz folder_to_compress
```