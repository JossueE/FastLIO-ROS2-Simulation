
# Ackermann SLAM Simulation (ROS 2 + Ignition Gazebo) 🏎️
**Author:** Jossue Espinoza <br>

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Ignition-Gazebo-orange.svg)](https://gazebosim.org/home)
[![SLAM](https://img.shields.io/badge/SLAM-Integration-brightgreen.svg)](https://github.com/SteveMacenski/slam_toolbox)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

This ROS 2 package is designed to simulate various environments and mobile robots in Ignition Gazebo.
It focuses on demonstrating an Ackermann-drive robotic system with integrated SLAM capabilities for mapping and autonomous navigation.

**NOTE:** this repo was tested on Ubuntu 22.04 LTS, with ROS 2 Humble and Gazebo Fortress (Ignition).

--- 

> [!IMPORTANT]
> This repository is based on the work of C. Mauricio Arteaga-Escamilla from Robótica Posgrado.
> **Contact email:** cmauricioae8@gmail.com
> **LinkedIn:** https://linkedin.com/in/cruz-mauricio-arteaga-escamilla/
> **YouTube:** https://www.youtube.com/channel/UCNmZp0rCuWxqaKVljny2zyg 


> [!IMPORTANT]
> To obtain a correct behavior of the sensors, the world.sdf file MUST be correctly set by adding the corresponding 'plugin' tag inside the 'world' tag. For more information. please refer to https://gazebosim.org/docs/latest/sensors/.


## 📚 Table of Contents
- [Installation](#)
    - [Pre-requisites and Ignition installation](#)
    - [Installing ROS 2 packages](#)
    - [Cloning this Repo](#)
- [Simulation](#)
    - [Launching the Robot in Gazebo](#)
    - [Teleoperating the Robot](#) 



---

## Installation

### Pre-requisites and Ignition installation

It is assumed that any distro of ROS 2 is already installed.
To avoid possible errors, please update your system and install the following ROS 2 dependencies.

> [!NOTE]
> You don’t need to modify $ROS_DISTRO; it’s a global environment variable.
> If the command doesn’t work, it probably means your ROS 2 installation is incomplete or not properly sourced.

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf2-* ros-$ROS_DISTRO-rviz-default-plugins
```

To install Ignition to work with ROS 2, run the following command:

```bash
sudo apt-get install ros-$ROS_DISTRO-ros-gz
```

The ros-gz package from source can be found here 
https://github.com/gazebosim/ros_gz/tree/humble

> [!IMPORTANT]
> Additionally, to be able to communicate our simulation with ROS 2, it is needed to use a package called 'ros_gz_bridge'. This package provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo transport. You can install this package by typing:

```bash
sudo apt-get install ros-$ROS_DISTRO-ros-ign-bridge
```

---
### Installing ROS 2 packages

To avoid possible errors, please update your system and install the following ROS 2 dependencies.

```
sudo apt-get update
```

ROS 2 dependencies for robot description:

```bash
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf2-* ros-$ROS_DISTRO-gazebo-* ros-$ROS_DISTRO-rviz-default-plugins ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-controller-manager
```

If the following error appears:<br>
_LookupError: Could not find the resource '<package_name>' of type 'packages'_

Try to install the corresponding ROS dependency with

```bash
sudo apt-get install ros-$ROS_DISTRO-<package-name>
```

For example:

```bash
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-gui
```

---

### Cloning this Repo

Please, paste this package in the src folder. Then:
```bash
cd ~/colcon_ws
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

If you already have all your dependencies, the console will return:<br>
#All required rosdeps installed successfully

**Note:** _This is made only once for the whole workspace._

Then, build colcon ws:
```bash
colcon build --packages-select ackermann_slam_sim --symlink-install
source install/setup.bash
```

**IMPORTANT:** This builds the package and sets a symbolic link to the python files (nodes and launch files). With this, re-build every time that a python file is modified, is not required.<br>
In ROS 2, launch files may be written in yaml, xml or python languages, but it is extremely recommended to use python. Also, the name of all launch files must finish with 'launch.py'. Otherwise, the file will not be recognized.

If some warnings appear, run `colcon build --packages-select ackermann_slam_sim --symlink-install` again and they will disappear.

---

## Simulation

### Launching the Robot in Gazebo

The `one_robot_ign_launch.py` file launches **Gazebo (Ignition)** using a predefined world and spawns the selected robot model automatically.

Inside the launch file, you’ll find configurable parameters such as:

```python
robot_model = 'ackermann' #Here you can add your URDF model defined in ackermann_slam_sim/urdf
robot_ns = 'r1' # Robot namespace (robot name)
pose = ['1.0', '0.0', '0.0', '0.0'] #Initial robot pose: x,y,z,th
robot_base_color = '0.0 0.0 1.0 0.95' #Ign and Rviz color of the robot's main body (rgba)
world_file = 'warehouse.sdf' # empty, warehouse
package_name = 'ackermann_slam_sim'
```

You can launch the simulation with:

```bash
ros2 launch ackermann_slam_sim one_robot_ign_launch.py
```
> [!NOTE]
> The first launch may take longer as Gazebo caches assets and loads world resources.

If you want to modify parameters such as the robot model, initial pose, world file, base color, or namespace, edit the `one_robot_ign_launch.py` file directly.

---

### Teleoperating the Robot

To teleoperate both the _differential_ and _omnidirectional_ mobile robot, use the package node:

```bash
ros2 run ackermann_slam_sim omni_teleop_keyboard.py
```

To use a namespace, to remap topics, services and node name, please use:

```bash
ros2 run ackermann_slam_sim omni_teleop_keyboard.py --ros-args -r __ns:=/r1
```

To publish a velocity from terminal:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1}, angular: {z: 0.3}}"
```
```bash
ros2 topic pub --once /r1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1}, angular: {z: 0.3}}"
```
To publish a velocity directly on a Ignition Topic from terminal:

```bash
ign topic -t "/model/r1/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5, y: 0.5}"
```
---