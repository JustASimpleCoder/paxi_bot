# Paxi - Package Taxi Robot
Paxi - "Package Taxi"

> An autonomous indoor ROS2 differential drive robot for collecting and delivering packages in office environments.

Built on modified hoverboard firmware with ROS2 Humble for control, sensor fusion, SLAM, and autonomous navigation. Paxi runs on a Jetson AGX Orin and is designed to navigate office spaces without human intervention.

The paxi_hardware package within this project (custom ROS2 robot hardware interface) is originally adapted from from [Alex Makarov's Robaka ROS1 hoverboard project](https://github.com/alex-makarov/robaka-ros), converted for ROS2 humble and updated to use C++17.

## Target
- Ubuntu 22.04
- tested on x86-64 and ARM64

## Hardware Overview
| Component | Details |
|---|---|
| **Drive** | Hoverboard BLDC motors with encoder feedback |
| **Mainboard MCU** | GD32F103RCT6 (ARM Cortex-M, STM32-compatible) |
| **Sideboard MCU** | GD32F130C6T6 (ARM Cortex-M, STM32-compatible) |
| **LiDAR** | RPLIDAR C1 |
| **IMU 1** | MPU-6050 6-axis (on sideboard PCB) |
| **IMU 2** | Slamtech 9-axis IMU Module|
| **Robot Computer** | NVIDIA Jetson AGX Orin Developer Kit (64GB) |
| **Drive Battery** | Li-ion 36V 4.4Ah (158.4Wh) |
| **Computer Battery** | Voltaic Systems 19V 4.5A |
| **Connections** | USB-TTL (hoverboard), USB-Serial (LiDAR) |
---

<!-- ```
Sideboard MCU                    Mainboard MCU                   Jetson AGX Orin
──────────────                   ─────────────                   ───────────────
MPU-6050 IMU        serial      Motor Control (FOC)   USB-TTL     ROS2 Humble
Madgwick Filter  ───────────►   Encoder Feedback    ───────────► paxi_hardware
Quaternions                      IMU Passthrough                 ros2_control
                                                                 SLAM Toolbox
                                                                 NAV2
``` -->

## Repository Structure
This repo (`paxi_bot`) contains the ROS2 workspace. External packages are pulled in via `vcstool`:

| Package | Source | Description |
|---|---|---|
| `paxi_bringup` | this repo | Launch files for all robot modes |
| `paxi_calibrate` | this repo | generates csv files for tareget RPM to feedback RPM |
| `paxi_data_analysis` | this repo | creates a linear regression model from generated csv data in paxi_calibrate|
| `paxi_gazebo` | this repo | gazebo simulation of robot (imcomplete) ||
| `paxi_hardware` | this repo | ROS2 hardware interface (adapted from [Robaka](https://github.com/alex-makarov/robaka-ros)) |
| `paxi_msgs` | this repo | custom ROS2 messages for paxi |
| `paxi_nav2` | this repo | custom Nav2 commander APIs (imcomplete) |
| `sllidar_ros2` | [fork](https://github.com/JustASimpleCoder/sllidar_ros2.git) | RPLIDAR C1 ROS2 driver |
| `slamkit_ros2` | [fork](https://github.com/JustASimpleCoder/slamkit_ros2) | Slamtec IMU ROS2 driver |

## Firmware

### **Hoverboard Firmware**
- **Mainboard Firmware (FOC)**  
  Modified from: [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)  
  My fork: [JustASimpleCoder/hoverboard-firmware-hack-FOC](https://github.com/JustASimpleCoder/hoverboard-firmware-hack-FOC)  
  - Handles motor control (PID BLDC).
  - Calculates encoder wheel speed in RPM from HAL sensor.
  - Receives IMU data from sideboard and motor command data over serial.
  - Sends feedback (IMU + encoder) back over USB-TTL serial with checksums.

### **Sideboard Firmware**  
  Modified from: [EFeru/hoverboard-sideboard-hack-GD](https://github.com/EFeru/hoverboard-sideboard-hack-GD/tree/main)  
  My fork: [JustASimpleCoder/hoverboard-sideboard-hack-GD](https://github.com/JustASimpleCoder/hoverboard-sideboard-hack-GD)  
  - Handles MPU 6050 IMU acceleration and angular velocity readings, and runs MadgwickAHRS algorithm to generate quaternions (see [Madgwick Orientation Filter](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html) and [Open Source Code](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/))
  - Sends IMU data (accel/gyro/quaternions) to the mainboard via serial communication (with checksums).  
---

## Features
- Two-wheel differential drive using hoverboard motors with ROS2 Differential Drive plugin.  
- Real-time wheel encoder and IMU feedback data.  
- Custom ROS2 hardware interface to send ROS2 control commands to hoverboard hardware.  
- LiDAR integration.
- Map generation using SLAM toolbox
- Autonomous Navigation using NAV2  
- Calibration tool to generate CSV files for target RPM compared to feedback RPM.
- Data analysis tool that implements a linear regression model to map target RPM to input PWM signals.
---

## System Requirements
- Ubuntu 22.04
- at least 16gb of RAM (tested with 16GB and 64GB of RAM), possibly less
- Hoverboard taken apart and MCU's flashed with the firmare descibed in the Firmware section above

## Setup
This assumes you already have installed ROS2 humble, see ROS2 wiki for details: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html). This also assumes you have flashed both my modified mainboard and sideboard firmware, see EFeru github wiki for flashing help: [EFeru Flashing Firmware](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Compiling-and-flashing-the-firmware).

### 1. Install prerquisites
vcs tool will help to pull in the repos descript in paxi.repos (in paxi_ws/src). This should come with a ros2 humble install but just in case you dont have it:
    ```bash
    sudo apt install python3-vcstool
    ```

### 2. Clone thev repository

```bash 
cd <path_to_project>
git clone https://github.com/JustASimpleCoder/paxi_bot.git
```

### 3. clone external packages from other repositories
'''bash
cd paxi_bot/paxi_ws
vcs import src < src/paxi.repos
'''

### 4. Add udev rules for slamkit_ros2 and sllidar: 
- sllidar_ros2 create udev rules script: 
```bash
cd <path_to_project>/paxi_ws/src/sllidar_ros2/scripts
sudo ./create_udev_rules.sh
```
- slamkit_ros2 create udev rules script: 
```bash
cd <path_to_project>/paxi_ws/src/slamkit_ros2/scripts
sudo ./add_udev.sh
```

### 5. Initialize SDK submodule in slamkit_ros2
The slamkit_ros2 package relies on Slamtec's sdk package which is managed by a submodule
```bash
cd src/slamkit_ros2
git submodule update --init

```
### 6. Install ROS2 Dependencies 
```bash
cd ~/<path_to_project>/paxi_bot/paxi_ws

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
> If rosdep misses anything, there is a fallback script at `paxi_bot/scripts/paxi_dependencies.sh`.

### 7. Build the workspace
```bash
colcon build
```
### 8. Source the built setup files and now you can launch the main bringup for a test (assuming the robot is built and flashed with the appropriate firmware)
```bash
source install/setup.bash 
ros2 launch paxi_bringup main_bringup.py
```

## Launch Files
Ensure you are in the paxi_ws and the project has been built

```bash
cd <path_to_project>~/paxi_bot/paxi_ws
```
#### Main bringup 
Launches everything to start the robot (robot state publisher, hardware interface, diff drive controller, cmd_vel remappings, lidar node, EKF node):
```bash
source install/setup.bash
ros2 launch paxi_bringup main_bringup.py
```
#### Live Display
Live display in RVIZ visualizing everything (LiDAR, IMU, Odom, TFs, Robot Model), to be view while robot is running (can be running on other machines as long as 'ROS_DOMAIN_ID' are the same on both machines)
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
Launches NAV2 stack with params from paxi_description/config/nav2_params.yaml:
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
## Quick Starts Scripts
These tmux-based scripts launch multiple nodes in split panes automatically.

You will need to create a map that works for your enviroment, use the live mapping script to create one.
### Live Mapping
Create a map of your environment using SLAM Toolbox:

Navigate to the scripts directory and run,
```bash
cd <path_to_project>/paxi_bot/scripts
./live_mapping_launch.bash
```
To move the robot manual for initial mapping you need to navigate to paxi_ws directory and run (on the robot machine or seperate machine):
```bash
cd <path_to_project>/paxi_bot/paxi_ws
source install/setup.bash
ros2 launch paxi_bringup manual_control.py
```
To visualize the robot while it is navigating and see the map it is creating, open another terminal and run (on the robot machine or seperate machine):
```bash
cd  <path_to_project>/paxi_bot/paxi_ws
source install/setup.bash
ros2 launch paxi_bringup live_mapping_display.py
```

To save the map you created, navigate to the the scripts directory and run:
```bash
cd <path_to_project>/paxi_bot/scripts
./save_nav2_map.sh <your_map_name>
```
Make sure to update the map filename or pass as an argument to the launch file in paxi_bringup/nav2.py

### Autonomous Navigation with RViz Display
Make sure you have a map that paxi can navigate with!

Navigate to the scripts directory and run on the robot machine:
```bash
cd <path_to_project>/paxi_bot/scripts
./nav2_launch_on_robot.bash
```

To visualize the robot while it is navigating and send NAV2 goals run (on the robot on seperate machine):
```bash
cd  <path_to_project>/paxi_bot/paxi_ws
source install/setup.bash
ros2 launch paxi_bringup rviz_nav2.py
```

## TODO

- [ ] Simple Commander API
- [ ] Behaviour Tree for indoor navigation
- [ ] Gazebo simulation