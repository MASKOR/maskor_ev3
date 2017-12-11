Content:

maskor_ev:
The core package. Contains libmaskor_ev3.so (based on ev3-dev package) and maskor_ev3_bobb3e_node, based on rosserial embedded linux libraries. Should be configured to cross-compile everything for ARM CPUs used on the EV3.

maskor_ev_bobb3e_nodes:
contains c++ ROS-Nodes that run on the ROS Master PC to make the robot do stuff (e.g. line-follower)

maskor_ev3_description:
contains URDF model and Meshes of the Bobb3e for visualization in RVIZ and/or Gazebo

maskor_ev3_messages:
contains own message definitions for all used sensors based on the EV3 users manual (Colorsensor, Gyrosensor, etc...)




