---
layout: default
title: TurtleBot 4 
parent: Tutorials
sort: 4
---
# TurtleBot 4 Hardware & Software Reference

The **TurtleBot 4** is a robotics platform for learning and development, built on the **iRobot® Create® 3** educational robot and powered by a **Raspberry Pi 4B**. This reference covers both the Standard and Lite models using the **ROS 2 Jazzy Jalisco** software stack.

---

## 1. Hardware Overview

The TurtleBot 4 is available in two configurations. While both share the same mobile base and compute, they differ in sensor payload and user interface capabilities.



### Core Technical Specifications

| Feature | TurtleBot 4 Lite | TurtleBot 4 Standard |
| :--- | :--- | :--- |
| **Dimensions** | 342 x 339 x 192 mm | 342 x 339 x 351 mm |
| **Weight** | 3.3 kg | 3.9 kg |
| **Max Payload** | 9 kg (Default) / 15 kg (Custom) | 9 kg (Default) / 15 kg (Custom) |
| **Max Speed** | 0.31 m/s (Safe) / 0.46 m/s (Raw) | 0.31 m/s (Safe) / 0.46 m/s (Raw) |
| **LIDAR** | RPLIDAR A1 | RPLIDAR A1 (Standard) / A2 (Optional) |
| **Camera** | OAK-D-Lite | OAK-D-Pro (IR/Active Stereo) |
| **User Interface** | 2 LEDs | OLED Display, 4 Buttons, 4 LEDs |
| **Battery** | 26 Wh Li-Ion (14.4V) | 26 Wh Li-Ion (14.4V) |

### User Power Output
- **Standard Model:** VBAT @ 1000mA, 12V @ 1000mA, 5V @ 2000mA, 3.3V @ 500mA.
- **Lite Model:** VBAT @ 1.9A (14.4V nominal). 5V and 3.3V available via Raspberry Pi GPIO.

## 2. Software Stack (ROS 2 Jazzy)

The TurtleBot 4 software environment is built for **Ubuntu 24.04 LTS** and **ROS 2 Jazzy Jalisco**.

### Core Requirements
- **OS:** Ubuntu 24.04.5 LTS (Noble Numbat).
- **Firmware:** iRobot® Create® 3 Firmware version **I.0.0** is required for Jazzy compatibility.
- **RMW Implementation:** FastRTPS (standard) or CycloneDDS.

### Key ROS 2 Packages
- **`turtlebot4_base`:** Manages the hardware interface (OLED, buttons, and Raspberry Pi/Create 3 communication).
- **`turtlebot4_bringup`:** Launch files for all sensors (LIDAR, OAK-D) and robot nodes.
- **`turtlebot4_navigation`:** Optimized Nav2 parameters using the **MPPI Controller** and SLAM Toolbox configurations.
- **`turtlebot4_description`:** URDF and mesh files for the physical robot model.
- **`turtlebot4_simulator`:** Simulation support for **Gazebo Harmonic**.

---

## 3. LED Ring Status (Create® 3 Base)

The LED ring on the mobile base provides the primary visual feedback for the robot's system state.

| Color & Pattern | Meaning | Action Required |
| :--- | :--- | :--- |
| **Solid White** | **Ready.** System is idle and connected. | None. You can now run ROS 2 nodes if the RPi is ready. |
| **Circling White** | **Booting.** Application is starting up. | **Wait.** This can take 2–5 minutes. |
| **Spinning Blue** | **AP Mode.** Create Base is a Wi-Fi hotspot. | Connect your laptop to `Create-XXXX`. |
| **Solid Blue** | **AP Connected.** Device is linked to Robot AP. | Complete the web setup at `192.168.10.1`. |
| **Solid Red** | **System Error.** Motor stall or cliff sensor. | Clear obstacles or debris from the wheels. |
| **Pulsing Red** | **Battery Low.** Needs charging. | Place the robot on the dock. |
<!-- | **Spinning Red** | **Critical Error.** Initialization failed. | **Hard Reboot.** Hold Power for 20 seconds. | -->
<!-- | **Pulsing Green** | **Charging.** Battery is filling. | None. Leave on dock until solid green. | -->

---

lidar wont work when charging

## 4. Power Management

The TurtleBot 4 uses two systems: the **Raspberry Pi 4** and the **Create® 3 Base**. Correct shutdown is essential to prevent SD card corruption.

### Startup
1. Place the robot on the charger dock.
2. Wait ~3 minutes for the robot to initialize (a white light rotates/pulses during the boot process):
   1. First, you will hear a chime <audio controls src="{{ '/assets/audio/cb_startup.wav' | relative_url }}"></audio> that indicates the Create Base is booted up
   2. Around 45 seconds later, you will hear a second chime <audio controls src="{{ '/assets/audio/robot_ready.wav' | relative_url }}"></audio> while a **fast purple light** rotating in the ring. This indicates the RPi and Create 3 are communicating, and the Robot is ready to use!

### Graceful Shutdown
> This procedure ensures a graceful shutdown of both the Raspberry Pi and the Create 3 base, preventing SD card corruption and hardware-level data loss.

1. Move the robot off the charger.
2. In your SSH session: `sudo shutdown -h now`.
3. Wait 30 seconds until the **LiDAR stops spinning**, then hold the **Power Button** (large ring button) for about 8 seconds until you hear a chime <audio controls src="{{ '/assets/audio/cb_shutdown.wav' | relative_url }}"></audio> and the LED turn off.

### LiDAR
The LiDar stops spinning when the robot is ready and docked. Once, undocked it will start spinning again and steam the data.

---

## 5. Robot Audio Cues

- **Create Base - Startup Chime** <audio controls src="{{ '/assets/audio/cb_startup.wav' | relative_url }}"></audio>
- **Robot Ready - Startup Chime** <audio controls src="{{ '/assets/audio/robot_ready.wav' | relative_url }}"></audio>
- **Low Battery Warning** <audio controls src="{{ '/assets/audio/low_battery.wav' | relative_url }}"></audio>
- **Create Base - Shutdown Chime** <audio controls src="{{ '/assets/audio/cb_shutdown.wav' | relative_url }}"></audio>

## 7. ROS Communication Issues

### 7.1. Time
Sometimes the time is out of sync. 
In your robot terminal:
Check time using timedatectl and update the time using the command: sync-ntp-time

## 6. Official Documentation & Links

- **[TurtleBot 4 User Manual (Official)](https://turtlebot.github.io/turtlebot4-user-manual/)**
- **[Jazzy Software & Navigation Guide](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)**
- **[iRobot® Create® 3 Education Portal](https://edu.irobot.com/what-we-offer/create3)**
- **[Luxonis OAK-D-Pro Tech Specs](https://docs.luxonis.com/projects/hardware/en/latest/pages/oak-d-pro.html)**
- **[TurtleBot GitHub Repositories](https://github.com/turtlebot)**