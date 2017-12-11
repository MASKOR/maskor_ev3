## Packages Description:

## maskor_ev3:
The core package. Contains libmaskor_ev3.so (based on ev3-dev package) and maskor_ev3_bobb3e_node, based on rosserial embedded linux libraries. The library needs to be deployed and the node must be executed on the EV3. This package should/can be configured to cross-compile everything for ARM CPUs.

## maskor_ev_bobb3e_nodes:
contains c++ ROS-Nodes that run on the ROS Master PC to make the robot do stuff (e.g. line-follower)

## maskor_ev3_description:
contains URDF model and 3D meshes of the Bobb3e for visualization in RVIZ and/or Gazebo

## maskor_ev3_messages:
contains ROS message definitions for all used sensors based on the EV3 users manual (Colorsensor, Gyrosensor, etc...)

## Installation
Clone the repository:
```sh
$ mkdir -p ~/ev3_ws/src
$ cd ~/ev3_ws/src
$ git clone https://github.com/MASKOR/maskor_ev3
```

Install all rosdeps (kinetic)
```sh
$ cd ~/ev3_ws/src/maskor_ev3/
$ ./install_rosdeps.sh
```

Install Cross-Compiler (optional but recommended)
```sh
sudo apt install g++-arm-linux-gnueabi
sudo apt install gcc-arm-linux-gnueabi
```

Compile
```sh
$ cd ~/ev3_ws/
$ catkin_make
```
## Basic Usage
Ensure that libmaskor_ev3.so and maskor_ev3_bobb3e_node from package maskor_ev3 are deployed on the EV3. After hat, run the following on the PC (not on EV3!):
```sh
$ roscore
```
```sh
$ rosrun rosserial_python serial_node.py tcp
```
```sh
$ roslaunch maskor_ev3_description robot_description.launch
```
After rosserial tcp server has been started, change to EV3 and run:
```sh
$ ./maskor_ev3_bobb3e_node <ip-of-your-ros-master>
```

You should now be able to send cmd_vel messages to control the Robot.



