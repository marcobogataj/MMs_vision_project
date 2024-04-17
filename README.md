# MMs_vision_project
A ROS repository with the aim of performing the bin picking of screws/bolts, more generally cylindrical objects. Consists in the integration of the ABB IRB120 manipulator, Robotiq Gripper 85mm and Zivid One+ Small camera inside the MoveIt framework, with custom perception and planning algorithms to carry out the task.

This work is the result of the master thesis by Marco Bogataj and Matteo Vitali at the Mechatronics and Mechanical Dynamics Lab (MDLAB) of the University of Bergamo located inside the Kilometro Rosso innovation district (2023/2024)

![Robotic cell image](/images/robotic_cell.png)

---

## Requirements

### ROS

This work was tested only on Ubuntu 20.04 with ROS Noetic. Follow the official [ROS installation instructions](http://wiki.ros.org/ROS/Installation) for your OS. Previous or future compatibility is not guaranteed.

### Zivid Driver

Follow the instructions for the [Zivid-ROS Driver](https://github.com/zivid/zivid-ros) installation. This is required for the integration of the Zivid One+ Small camera.

### ABB Driver 

Follow the instructions at [ABB Driver](https://github.com/ros-industrial/abb_driver) and in the [ROS Wiki](http://wiki.ros.org/abb_driver) for the installation both on the Linux PC and the ABB controller.

If a module import error appears on the Flex Pendant interface about "LINKEDM" add `MODULE LINKEDM (SYSMODULE)` in the first lines of code inside `HOME/linked_m.sys`.

### Robotiq Driver 

No official robotiq driver is present from ROS-I, so the gripper driver package 'robotiq_2finger_grippers' is given in the main folder of the repository, which is based and comes from the [Danfoa Robotiq Driver](https://github.com/Danfoa/robotiq_2finger_grippers). Head there to know more about its requisites.

### Support packages

Both the robotic arm and the gripper require support packages to load robot URDF model, meshes, etc. onto the parameter server and therefore work within the MoveIt framework. These are respectevely called `abb_experimental` which contains folders with the IRB120 description, and `robotiq_85_description`. They were slightly tuned, particularly the IRB120 URDF was modified compared to the original one inside [ABB Experimental](https://github.com/ros-industrial/abb_experimental) to match the End Effector orientation inside Rviz to the one in Robot Studio.

## Installation

Clone the MM VISION PROJECT into the src directory:

```bash
cd ~/catkin_ws/src
git clone https://github.com/marcobogataj/mms_vision_project
```
Finally build the project using 

```bash
cd ~/catkin_ws/src
catkin build
```

## Getting Started

To perform the bin picking task open four different command line windows, and type in all the windows:

```bash
cd ~/catkin_ws && source devel/setup.bash
```

This is to set up the catkin environment.

Then in the first window type:

```
roslaunch roslaunch irb120_robotiq_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=XXX.XXX.XXX.X

```

To connect to the robot controller locate on the network at the IP -> XXX.XXX.XXX.X

To simulate only, type:

```
roslaunch roslaunch irb120_robotiq_moveit_config moveit_planning_execution.launch sim:=true

```

Keep in mind that while the robotiq arm has a robot simulator, the robotiq gripper cannot be simulated with the driver provided.


To launch the zivid driver and custom samples type in a second command window:

```
roslaunch zivid_capture capture_no_rviz.launch

```

In the third window, run the point cloud processor for finding cylinder in the scene by typing:

```
rosrun pcl_processing screw_perception 

```

Lastly, the bin picking manager node with:

```
rosrun motion_planning bin_picking_manager

```

Interact with this last node command window to perform actions.
