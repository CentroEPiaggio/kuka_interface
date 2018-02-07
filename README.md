# kuka_interface
Simple C++/ROS/Matlab interface to send commands to kuka_lwr (https://github.com/CentroEPiaggio/kuka-lwr) controlled robots.

Requirements:
===
- A PC with Ubuntu 16.04 and ROS kinetic (also other version should work)
- A PC (can be the same one) with MATLAB R2016b at least

Installation:
===
In a terminal:
- mkdir -p ~/catkin_ws/src
- cd ~/catkin_ws/src
- catkin_init_workspace
- git clone https://github.com/CentroEPiaggio/kuka_interface.git
- cd ~/catkin_ws
- catkin_make

Matlab
===
In the *matlab_to_ros* folder you can find some simulink schemes. Using these you can send commands to the nodes stored in the *kuka_interface_pkg*.

Setup:
---
In the command Window of matlab:
- rosinit('http://**ROS_MASTER_IP**:11311'), substitute **ROS_MASTER_IP** with the IP of the pc hosting the roscore

Open one of the simulink schemes and use them to send commands to ROS.

To finish:
- rosshutdown

C++/ROS
===
Using one of the nodes in the *kuka_interface_pkg* you can send command directly to a robot using the kuka_lwr software in position control.

to start the interface type in a terminal (after the robot is started):
- roscore
- roslaunch kuka_interface manager.launch

now commands can be sent using matlab, or, as an example:
- rosrun kuka_interface homing