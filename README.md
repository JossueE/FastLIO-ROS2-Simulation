
# FastLIO-ROS2-Simulation (ROS 2 + Ignition Gazebo) 🏎️
**Author:** Jossue Espinoza <br>

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Ignition-Gazebo-orange.svg)](https://gazebosim.org/home)
[![SLAM](https://img.shields.io/badge/SLAM-Integration-brightgreen.svg)](https://github.com/SteveMacenski/slam_toolbox)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

This ROS 2 package provides a complete simulation framework for mobile robots in **Ignition Gazebo**.  
It acts as a bridge between simulated environments and [**FAST_LIO**](https://github.com/Ericsii/FAST_LIO_ROS2), enabling seamless integration of **LiDAR**, **IMU**, and **vehicle models**.

Within this repository, you’ll find everything needed to:

- 🛰️ Simulate different **LiDARs**, **vehicles**, and **sensors**.  
- 🔄 **Synchronize IMU and LiDAR** data for accurate mapping.  
- 🗺️ Generate and **save maps** using FAST LIO.  
- 🤖 Add custom **mobile robots** to your simulations.  
- 🧠 Test and evaluate **SLAM algorithms** in virtual environments.

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
    - [Cloning FAST_LIO adapted to work with ROS2](#)
    - [Localization (adapted for ROS 2) ](#)
    - [Cloning this Repo](#)
- [Configuration](#)
    - [Configure The Simulation](#)
        - 

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

If you already have all your dependencies, the console will return: <br>
```bash
All required rosdeps installed successfully
```

**Note:** _This is made only once for the whole workspace._

---

### Cloning FAST_LIO -> Mapping (adapted for ROS 2) 
Please refer to the official documentation at  
[FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)  
for detailed installation and configuration instructions.

> [!IMPORTANT]  
> **Dependencies are mandatory for this algorithm to work properly.**  
> Make sure the following libraries are installed before building:
> 
> - **PCL ≥ 1.8** — Follow the [PCL Installation Guide](https://pointclouds.org/downloads/#linux)  
> - **Eigen ≥ 3.3.4** — Follow the [Eigen Installation Guide](http://eigen.tuxfamily.org/index.php?title=Main_Page)

This package follows a **plug-and-play architecture**.  
That means you can use the **same package** both in simulation and with your **real-world LiDAR** —  
without modifying or affecting the original source code. 

This package also requires a subpackage called **`livox_ros_driver2`**,  
which is essential when working with **Livox AVIA LiDARs** in real-world applications.

If you **do not plan to use a real LiDAR from Livox**, you can safely **comment out all related dependencies** —  
the package will still compile and run correctly in simulation mode.

However, if it’s not a problem for your setup, it’s recommended to **keep the original configuration**  
so that the system remains fully compatible with both **simulation** and **real-world** LiDAR setups.

### Localization (adapted for ROS 2) 


██╗░░░░░░█████╗░░█████╗░██████╗░██╗███╗░░██╗░██████╗░
██║░░░░░██╔══██╗██╔══██╗██╔══██╗██║████╗░██║██╔════╝░
██║░░░░░██║░░██║███████║██║░░██║██║██╔██╗██║██║░░██╗░
██║░░░░░██║░░██║██╔══██║██║░░██║██║██║╚████║██║░░╚██╗
███████╗╚█████╔╝██║░░██║██████╔╝██║██║░╚███║╚██████╔╝
╚══════╝░╚════╝░╚═╝░░╚═╝╚═════╝░╚═╝╚═╝░░╚══╝░╚═════╝░

### Cloning this Repo

Clone this package inside the `src` directory of your ROS 2 workspace.  
Replace **`colcon_ws`** with the name of your own workspace folder.

```bash
cd ~/colcon_ws/src
git clone https://github.com/JossueE/FastLIO-ROS2-Simulation.git
cd ..
```

Then, build colcon ws:
```bash
colcon build --packages-select ackermann_slam_sim --symlink-install
source install/setup.bash
```

> [!NOTE]
> This builds the package and sets a symbolic link to the python files (nodes and launch files). With this, re-build every time that a python file is modified, is not required.<br>

If some warnings appear, run `colcon build --packages-select ackermann_slam_sim --symlink-install` again and they will disappear.

---
## Configuration

### Configure The Simulation

The `one_robot_ign_launch.py` file launches **Gazebo (Ignition)** using a predefined world and spawns the selected robot model automatically.

Inside the launch file, you’ll find configurable parameters such as:

```python
robot_model = 'ackermann' #Here you can add your URDF model defined in ackermann_slam_sim/urdf/
robot_ns = 'r1' # Robot namespace (robot name) ----> RViz is set for this value, Do not move unless you need.
pose = ['1.0', '0.0', '0.0', '0.0'] #Initial robot pose: x,y,z,th
robot_base_color = '0.0 0.0 1.0 0.95' #Ign and Rviz color of the robot's main body (rgba)
world_file = 'depot.sdf' # empty, depot -----> This is the world of our simulation, is defined in ackermann_slam_sim/worlds/
package_name = 'ackermann_slam_sim'
```
> [!IMPORTANT]
> They are preconfigured for specific environments and nodes.
> Changing them without understanding their dependencies may cause the system to break.

### Adding New Robots and Worlds

You can easily **add a new robot model** by creating a **URDF** description in  
`ackermann_slam_sim/urdf/`, preferably in **`.xacro`** format for easier parameterization and reuse.

To include a new environment, simply **add your world file** in  
`ackermann_slam_sim/worlds/` using the **`.sdf`** format.

Once added, you can select them in the launch file by updating:
```python
robot_model = '<your_robot_name>'
world_file = '<your_world_name>.sdf'
```

### Configure your LiDAR (URDF/Xacro + Ignition)

This repo mounts a LiDAR on `base_link` and spawns an Ignition (Gazebo) ray sensor.  
You can tune **pose**, **FOV**, **resolution**, **rate**, **range**, **noise**, and the **topic**.

Key parameters inside `<gazebo><sensor … type='gpu_lidar'>`:
- `update_rate` (Hz): sensor frequency.
- `<scan>/<horizontal|vertical>`: `samples`, `min_angle`, `max_angle`, `resolution`.
- `<range>`: `min`, `max`, `resolution`.
- `<noise>`: `type`, `mean`, `stddev`.
- `<topic>`: Ignition/Gazebo sensor topic name (bridge it to ROS 2 if needed).
- `<ignition_frame_id>`: TF frame for the point cloud.

> **Tip:** Use **`gpu_lidar`** if you have GPU available (faster). Use **`lidar`** for CPU-only.

In this repo I put a general example of a LIDAR but if you want an specific one. Here are some tips to configure your lidar about what you need.

The `<update_rate> 10 </update_rate>` is set to 10Mhz cause the most part of Lidars works at this frecuency, but you can pot yours here.

Then we have the block code `<scan> ... </scan>` here you are going to find to parts `<Horizontal>` and `<Vertical>` compoused by:

```html

  <samples>360</samples>             
  <resolution>1</resolution>
  <min_angle>${-PI}</min_angle>
  <max_angle>${PI}</max_angle>


   <samples>32</samples>     
   <resolution>1</resolution>
   <min_angle>${-PI/4}</min_angle> <!-- -45 deg -->
   <max_angle>${PI/4}</max_angle> <!-- +45 deg -->
```

Then we have 

<range>
    <min>0.1</min>
    <max>100.0</max>
    <resolution>0.017453</resolution>
</range>

And finaly to simulate noise

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>

> [!IMPORTANT]
> Do not modify the topic unless it might be necesary. 



---

## Simulation

### Launching the Robot in Gazebo


You can launch the simulation with:

```bash
ros2 launch ackermann_slam_sim one_robot_ign_launch.py
```
> [!NOTE]
> The first launch may take longer as Gazebo caches assets and loads world resources.

If you want to modify parameters such as the robot model, initial pose, world file, base color, or namespace, edit the `one_robot_ign_launch.py` file directly.

> [!IMPORTANT]
> Is important to say that please do not modify the name of the topics, Unless you really understand what are you doing. As a result that this package is totally compatible with ... and is important be congruent. 

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
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1}, angular: {z: 0.3}}"
```
To publish a velocity directly on a Ignition Topic from terminal:

```bash
ign topic -t "/model/r1/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5, y: 0.5}"
```
---