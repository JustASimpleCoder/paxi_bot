# paxi_bot
Paxi - "Package Taxi", a ROS2 robot for collecting packages in a indoor office space.

This project is built on modified hoverboard firmware using ROS2 for control, sensor fusion, SLAM and navigation. It is designed for an indoor robot that can autonomously navigate and collect packages inside an office.

The paxi_hardware package within this project (custom ROS2 robot hardware interface) is originally adapted from from [Alex Makarov's Robaka ROS1 hoverboard project](https://github.com/alex-makarov/robaka-ros), converted for ROS2 humble and updated to use C++17.

## Requirements 
- ROS2 Humble 
- Ubuntu 22.04
- Hoverboard
- at least 16gb of RAM (tested with 16GB and 64GB of RAM), possibly less

## Target
- Ubuntu 22.04
- tested on x86-64 and ARM64

## Other Repositories Utilized

### 1. Hardware
  - Mainboard: GD32F103RCT6 (STM32 alternative) ARM Cortex-M MCU 
  - Sideboard: GD32F130C6T6 (STM32 alternative) ARM Cortex-M MCU
  - Motors: Two BLDC motors (hoverboard wheels) with encoder wheel velocity feedback
  - Lidar: RPLIDAR C1
  - IMU: MPU-6050 that is on the sideboard's PCB
  - Mainboard MCU Battery: Li-ion battery 36V 4.4AH 158.4WH
  - Robot Machine: NVIDIA Jetson AGX ORIN Developer kit (64GB)
  - Robot Machine Battery: Voltaic Systems battery pack 19V 4.5A
  - Connections: USB-to-TTL converter for hoverboard mainboard to robot machine and USB-to-Serial converter for RPLidar to robot machine
---
### 2. Hoverboard Firmware
- **Mainboard Firmware (FOC)**  
  Modified from: [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)  
  My fork: [JustASimpleCoder/hoverboard-firmware-hack-FOC](https://github.com/JustASimpleCoder/hoverboard-firmware-hack-FOC)  
  - Handles motor control (PID BLDC).
  - Calculates encoder wheel speed in RPM from HAL sensor.
  - Receives IMU data from sideboard and motor command data over serial.
  - Sends feedback (IMU + encoder) back over USB-TTL serial with checksums.

- **Sideboard Firmware**  
  Modified from: [EFeru/hoverboard-sideboard-hack-GD](https://github.com/EFeru/hoverboard-sideboard-hack-GD/tree/main)  
  My fork: [JustASimpleCoder/hoverboard-sideboard-hack-GD](https://github.com/JustASimpleCoder/hoverboard-sideboard-hack-GD)  
  - Handles MPU 6050 IMU acceleration and angular velocity readings, and runs MadgwickAHRS algorithm to generate quaternions (see [Madgwick Orientation Filter](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html) and [Open Source Code](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/))
  - Sends IMU data (accel/gyro/quaternions) to the mainboard via serial communication (with checksums).  
---

### 3. SLAM & LiDAR Integration

- **Slamtec RPLIDAR (ROS2)**  
  Modified from: [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)  
  My fork: [JustASimpleCoder/sllidar_ros2](https://github.com/JustASimpleCoder/sllidar_ros2.git)  
  - Only minor modifications: removed CMake warnings as it was annoying me and making it difficult to diagnose issues with other packages while building.  
  - Provides ROS2 nodes for LiDAR scanning and publishing ROS2 LaserScan msgs.  
---

## Features
- Two-wheel differential drive using hoverboard motors with ROS2 Differential Drive plugin.  
- Real-time wheel encoder and IMU feedback data.  
- Custom ROS2 hardware interface to send ROS2 control commands to hoverboard hardware.  
- LiDAR integration.
- Map generation using SLAM toolbox
- Autonomous Navigation using NAV2  
- Claibration tool to generate CSV files for target RPM compared to feedback RPM.
- Data analysis tool that implements a linear regression model to map target RPM to input PWM signals.
---

## Setup

This assumes you already have installed ROS2 humble, see ROS2 wiki for details: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html). This also assumes you have flashed both my modified mainboard and sideboard firmware, see EFeru github wiki for flashing help: [EFeru Flashing Firmware](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Compiling-and-flashing-the-firmware).

Move to (or create) the directory you wish to clone this project into and clone this repository (with submodules)

```bash 
cd <path_to_project>
git clone --recurse-submodules https://github.com/JustASimpleCoder/paxi_bot.git
```
If you already cloned without the --recurse-submodules tag above then make sure to run (pulls my sllidar_ros2)

```bash
cd paxi_bot
git submodule init
git submodule update
```
Install Dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
Build the workspace
```bash
colcon build
```
Source the built setup files and now you can launch the main bringup for a test (assuming the robot is built and flashed with the appropriate firmware)
```bash
source install/setup.bash 
ros2 launch paxi_bringup main_bringup.py
```
Note: Rosdep should pull all dependencies, but in case it fails to grab stuff from a submodule, there is a script to install ros dependencies located in paxi_bot/scripts, which you can run as (be sure to check what is in it before running!):
```bash
cd <directory_where_repo_is_cloned>/scripts
sudo ./paxi_dependencies.sh
```

## How to use
Go to the directory where you cloned paxi_bot and enter its workspace
```bash
cd <directory_where_repo_is_cloned>~/paxi_bot/paxi_ws
```
Then you can launch any of the following summarized below. 
#### Main bringup 
Launches everything to start the robot (robot state publisher, hardware interface, diff drive controller, cmd_vel remappings, lidar node, EKF node):
```bash
source install/setup.bash
ros2 launch paxi_bringup main_bringup.py
```
#### Live Display
Live display in RVIZ visualizing everything (LiDAR, IMU, Odom, TFs, Robot Model), to be view while robot is running (can be running on other machines as long as ROS_DOMAIN_ID are the same on both machines)
```bash
source install/setup.bash
ros2 launch paxi_bringup live_display.py
```
#### Static Display
static display (using static transforms) to visualize URDF files in RVIZ:
```bash
source install/setup.bash
ros2 launch paxi_bringup static_display.py
```
#### Manual Control
Manual controller with the standard teleop keyboard commands to move Paxi around. It opens in a new terminal window:
```bash
source install/setup.bash
ros2 launch paxi_bringup manual_control.py
```
#### NAV2 stack launch 
Launches NAV2 stack with the NAV2 params from paxi_description/config/nav2_params.yaml:
```bash
source install/setup.bash
ros2 launch paxi_bringup nav2.py
```
#### RViz nav2 launch 
Launches RViz (must have launch nav2.py or running nav2 stack) with the NAV2 plugin to manually send navigation goals:
```bash
source install/setup.bash
ros2 launch paxi_bringup rviz_nav2.py
```
## Quick Starts
These scripts below help simplify multiple launches by using a tmux session and launching each launchfile into their own pane. 

You will need to create a map that works for your enviroment, use the live mapping script to create one.
### Live Mapping
After the main paxi bringup, this will start the online async mapping from the slam_toolbox package, the navigation launch from nav2, rviz to visualize the mapping & robot urdf and the standard teleop twist control. You can manually control it using the teleop twist keyboard. Make sure you save your map that you create before stopping the tmux session!

Navigate to the scripts directory and run,
```bash
cd <directory_where_repo_is_cloned>/paxi_bot/scripts
./live_mapping_launch.bash
```
To move the robot manual for initial mapping you need to navigate to paxi_ws directory and run (on the robot machine or seperate machine):
```bash
cd <directory_where_repo_is_cloned>/paxi_bot/paxi_ws
source install/setup.bash
ros2 launch paxi_bringup manual_control.py
```
To visualize the robot while it is navigating and see the map it is creating, open another terminal and run (on the robot machine or seperate machine):
```bash
cd  <directory_where_repo_is_cloned>/paxi_bot/paxi_ws
source install/setup.bash
ros2 launch paxi_bringup live_mapping_display.py
```

To save the map you created, navigate to the the scripts directory and run:
```bash
cd <directory_where_repo_is_cloned>/paxi_bot/scripts
./save_nav2_map.sh <your_map_name>
```
Make sure to update the map filename or pass as an argument to the launch file in paxi_bringup/nav2.py

### Autonomous Navigation with RViz Display
After the main paxi bringup, this will start the online async mapping from the slam_toolbox package, the navigation launch from nav2, rviz to visualize the mapping & robot urdf and the standard teleop twist control. You can manually control it using the teleop twist keyboard. Make sure you save your map that you create before stopping the tmux session!

Navigate to the scripts directory and run on the robot machine:
```bash
cd <directory_where_repo_is_cloned>/paxi_bot/scripts
./nav2_launch_on_robot.bash
```

To visualize the robot while it is navigating and send NAV2 goals run (on the robot on seperate machine):
```bash
cd  <directory_where_repo_is_cloned>/paxi_bot/paxi_ws
source install/setup.bash
ros2 launch paxi_bringup rviz_nav2.py
```

## TODO

- Simple Commander API
- Behviour Tree for Indoor Navigating
- Gazebo Simulations