# paxi_bot
Paxi - "Package Taxi", a ROS2 robot for collecting and moving around packages.

This project is built on modified hoverboard firmware and SLAM sensor integration, using ROS2 for control, sensor feedback, and navigation. It is designed for experimentation with autonomous navigation, mapping, and sensor fusion.

---

## Project Structure

### 1. Hoverboard Firmware

- **Sideboard Firmware (GD MCU)**  
  Modified from: [EFeru/hoverboard-sideboard-hack-GD](https://github.com/EFeru/hoverboard-sideboard-hack-GD/tree/main)  
  My fork: [JustASimpleCoder/hoverboard-sideboard-hack-GD](https://github.com/JustASimpleCoder/hoverboard-sideboard-hack-GD)  
  - Handles motor control, IMU reading, and communication over serial.  
  - Includes tweaks and custom protocol integration for ROS2 communication.  

- **Mainboard Firmware (FOC)**  
  Modified from: [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)  
  My fork: [JustASimpleCoder/hoverboard-firmware-hack-FOC](https://github.com/JustASimpleCoder/hoverboard-firmware-hack-FOC)  
  - FOC (Field-Oriented Control) motor algorithms for precise speed/torque control.  
  - Adapted to work seamlessly with the sideboard protocol for sensor feedback.

---

### 2. SLAM & LIDAR Integration

- **Slamtec RPLIDAR (ROS2)**  
  Modified from: [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)  
  My fork: [JustASimpleCoder/sllidar_ros2](https://github.com/JustASimpleCoder/sllidar_ros2.git)  
  - Only minor modifications: removed CMake warnings.  
  - Provides ROS2 nodes for LIDAR scanning and publishing to the robot’s navigation stack.  

---

## Features

- Two-wheel differential drive using hoverboard motors.  
- Real-time IMU feedback via modified sideboard firmware.  
- ROS2-based control with support for custom nodes, sensor fusion, and navigation.  
- LIDAR integration for SLAM and autonomous mapping.  

---

## Setup

1. Clone all necessary repositories:

```bash
git clone https://github.com/JustASimpleCoder/hoverboard-sideboard-hack-GD.git
git clone https://github.com/JustASimpleCoder/hoverboard-firmware-hack-FOC.git
git clone https://github.com/JustASimpleCoder/sllidar_ros2.git


