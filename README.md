# paxi_bot
Paxi - "Package Taxi", a ROS2 robot for collecting and moving around packages.

This project is built on modified hoverboard firmware and SLAM sensor integration, using ROS2 for control, sensor feedback, and navigation. It is designed for experimentation with autonomous navigation, mapping, and sensor fusion.
The paxi_hardware package within this project (ROS2 robot hardware) is originally adapted from from [Alex Makarov's Robaka ROS1 hoverboad project](https://github.com/alex-makarov/robaka-ros), converted into a ROS2 humble (C++17).


## Requirments 
- ROS2 humble 
- Ubuntu 22.04
- Hoverboard parts
## Target

ROS2 humble on Ubuntu 22.04, built with ROS's build system (gcc under the hood), tested on x86-64 and ARM64

## Project Structure

### 1. Hardware
  - Mainboard: GD32F103RCT6 MCU with MPU-6050
  - Sideboard: GD32F130C6T6 MCU
  - Motors: Two BLDC motors (hoverboard wheels) with encoder feedback information
  - Lidar: RPILIDAR C1
  - IMU: MPU-6050 that is on the sideboard's PCB
---
### 2. Hoverboard Firmware

- **Sideboard Firmware (GD MCU)**  
  Modified from: [EFeru/hoverboard-sideboard-hack-GD](https://github.com/EFeru/hoverboard-sideboard-hack-GD/tree/main)  
  My fork: [JustASimpleCoder/hoverboard-sideboard-hack-GD](https://github.com/JustASimpleCoder/hoverboard-sideboard-hack-GD)  
  - Handles MPU 6050 IMU acceleration and angular velocity readings, and runs MadgwickAHRS algorithm to generate quaternions (see [Madgwick Orientation Filter](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html) and [Open Source Code](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/))
  - Sends IMU data (accel/gyro/quaternions) to the mainboard via serial communication (with checksums).  

- **Mainboard Firmware (FOC)**  
  Modified from: [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)  
  My fork: [JustASimpleCoder/hoverboard-firmware-hack-FOC](https://github.com/JustASimpleCoder/hoverboard-firmware-hack-FOC)  
  - Handles motor control (PID BLDC).
  - Calculates encoder wheel speed in RPM.
  - Receives IMU data from sideboard and motor command data over serial.
  - Sends feedback (IMU + encoder) back over USB-TTL serial with checksums.
---

### 3. SLAM & LIDAR Integration

- **Slamtec RPLIDAR (ROS2)**  
  Modified from: [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)  
  My fork: [JustASimpleCoder/sllidar_ros2](https://github.com/JustASimpleCoder/sllidar_ros2.git)  
  - Only minor modifications: removed CMake warnings as it was annoying me and making it difficult to diagnos issues with other packages while building.  
  - Provides ROS2 nodes for LIDAR scanning and publishing ROS2 LaserScan msgs.  
---

## Features
- Two-wheel differential drive using hoverboard motors.  
- Real-time IMU feedback via modified sideboard firmware.  
- ROS2-based control with support for custom nodes, sensor fusion, and navigation.  
- LIDAR integration for SLAM and autonomous mapping.  
---

## Setup

 Install ros2 humble, see ros2 wiki for details: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

Move to (or create) the directory you wish to clone this project into andlone this repository (with submodules)

```bash 
cd <pyour_path_to_project>
git clone --recurse-submodules https://github.com/JustASimpleCoder/paxi_bot.git
```
If you already cloned without the --recurse-submodules tag above then make sure to run

```bash
cd paxi_bot
git submodule init
git submodule update
```
This pulls in my sllidar_ros2 automatically. 

Install Dependencies
```bash
rosdep update
rosdep install --from-paths src --igrnore-src -r -y
```
Build the workspace
```bash
rosdep update
rosdep install --from-paths src --igrnore-src -r -y
```
Source the built setup files and now you can launch the main bringup for a test
```bash
source install/setup.bash 
ros2 launch paxi_bringup main_bringup.py
```

Note: Rosdep should pull all dependencies, but in case it fails to grab stuff from a submodule, there is a script to install ros depencdies located in paxi_bot/scripts, which you can run as (be sure to check what in it before running!):
```bash
  cd ~/paxi_bot/scripts
  sudo ./paxi_dependendencies.bash
```

## How to use
Go to the directory where you clones paxi_bot and enter its workspace
```
cd ~/paxi_bot/paxi_ws
```
Then you can launch any of the following sumamrized below. 
#### Main bringup 
Launches everything to start the robot, hardware interface, diff drive controller, lidar node, EKF node, nav2
```bash
source install/setup.bash
ros2 launch paxi_bringup main_bringup.py
```
#### Live Display
Live display in RVIZ visualizing everything (Lidar, IMU, Odom, TFs, ROobot Model), to be view while robot is running (can be running on other machines as long as ROS_DOMAIN_ID are the same on both machines)
```bash
source install/setup.bash
ros2 launch paxi_bringup live_display.py
```
#### Static Display
static display (using static transforms) to visualize URDF files in RVIZ
```bash
source install/setup.bash
ros2 launch paxi_bringup static_display
```