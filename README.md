# GH360 Packages for ROS2 Foxy

## Setup
### ROS Noetic Workspace
* Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [create a workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace) with the name **gh360_ws**
* Clone the [gh360_control](https://github.com/LauEls/gh360_control) package into the gh360_ws/src/
* Build the workspace
* Install pyserial:
  ```
  pip install pyserial
  ```
* Install rosserial_python:
  ```
  sudo apt install ros-noetic-rosserial-python
  ```
* Give permission to communicate with serial devices:
  ```
  sudo gpasswd --add ${USER} dialout
  ```
* Restart Ubuntu

You can test if this part of the installation worked by connecting the USB cable of the arm and running the following commands:
```
source gh360_ws/devel/setup.bash
roslaunch gh360_control encoder_manager.launch
```
In another terminal run:
```
source gh360_ws/devel/setup.bash
rostopic list
```
You should see the following topics: /arm1/Shoulder_Encoders, /arm1/UpperArm_Encoders, /arm1/LowerArm_Encoders

### ROS Foxy Workspace
* Install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) and [create a workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) with the name **ros2_gh360_ws**
* Clone the following packages into ros2_gh360_ws/src:
  * [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/foxy-devel) 
  * [gh360](https://github.com/LauEls/gh360)
  * [ros2_aruco](https://github.com/LauEls/ros2_aruco)
  * realsense-ros
  * rosbag2
 

* Install rosserial_python 
* Setup ROS1 bridge

### ROS1 Bridge
* Create a workspace with the name **bridge_ws**.
* Clone the [ros1_bridge](https://github.com/ros2/ros1_bridge/tree/foxy) package in the src folder.
* Follow the [build from source](https://github.com/ros2/ros1_bridge/tree/foxy?tab=readme-ov-file#building-the-bridge-from-source) instructions.

To test the ros bridge use the following commands:
* Terminal 1:
  ```
  source gh360_ws/devel/setup.bash
  roslaunch gh360_control encoder_manager.launch
  ```
* Terminal 2:
  ```
  source bridge_ws/install/setup.bash
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
  ```
* Terminal 3:
  ```
  source ros2_gh360_ws/install/setup.bash
  ros2 topic list
  ```
In Terminal 3 you should see the following topics: /arm1/Shoulder_Encoders, /arm1/UpperArm_Encoders, /arm1/LowerArm_Encoders
