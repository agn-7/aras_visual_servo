INTRODUCTION
---------------

The aim of visual servoing is tracking an object by the feedback information of the vision sensor. There are different techniques for controlling the system, in this work, Image based visual servoing has been chosen. 
The goal of propsed system is tracking the featureless object by the kernel and moment measurement.
The configuration of the system is eye-in-hand. 


OVERVIEW
--------------

The main packages are:

 - aras_visual_servo_camera

 - aras_visual_servo_controller

 - aras_visual_servo_gazebo

 - aras_teleop


INSTALL
---------------

Clone this project on src folder in your ROS-Workspace (~/catkin_ws/src/) then do make in workspace:
```bash
:~/catkin_ws/src$ git clone https://github.com/agn-7/aras_visual_servo.git
:~/catkin_ws$ catkin_make
```

Install Dependencies:
```bash
sudo apt-get install ros-<ros-distro>-joint-state-controller
sudo apt-get install ros-<ros-distro>-effort-controllers
sudo apt-get install ros-<ros-distro>-position-controllers
sudo apt-get install ros-<ros-distro>-gazebo-ros-control
```

RUN:
---------------

```bash
roslaunch aras_visual_servo_gazebo aras_visual_servo.launch
roslaunch aras_visual_servo_gazebo aras_visual_servo_nodes.launch
```
### For moving the arm with arrow keys run the following line:

```bash
rosrun aras_teleop mover.py
```

AUTHORS
---------------

Babak Sistani Zadeh Aghdam <br>
Javad Ramezanzadeh <br>
Parisa Masnadi <br>
Ebrahim Abedloo <br>
Hamid D Taghirad (supervisor) <br>
Benyamin Jafari (Forked this repo) <br>



