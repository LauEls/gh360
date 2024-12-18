# GH360 Packages for ROS2 Foxy

## Setup
### ROS Noetic Workspace
* Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [create a workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace) with the name **gh360_ws**
* Clone the [gh360_control](https://github.com/LauEls/gh360_control) package into the gh360_ws/src/
* Install pyserial:
  ```
  pip install pyserial
  ```
* Install rosserial_python:

### ROS Foxy Workspace
* Install ROS2 Foxy and create a workspace with the name ros2_gh360_ws
* Clone the following packages into the ros2_gh360_ws:
  * DynamixelSDK 
  * gh360
  * ros2_aruco
  * realsense-ros
  * rosbag2
 

* Install rosserial_python 
* Setup ROS1 bridge
