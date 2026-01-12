---
layout: default
title: "The Foundation: Linux & The Evolution of ROS"
parent: Tutorials
sort: 1
---
# The Foundation: Linux, Distros & ROS
> Estimated Reading Time: 15 mins

Before diving into robotic algorithms, you need to understand the "brain" (Linux) and the "language" (ROS) we use to communicate with it.
---

## 1. What is Linux?
Linux is a family of open source Unix-like operating systems based on the Linux kernel. Unlike Windows, which is designed for general consumer use, Linux is the backbone of the internet, servers, and robotics.

### Why do we use Linux for Robotics?
- **The Terminal:** Command-line control is faster and more scriptable than a mouse-driven GUI.
- **Open Source:** You can modify the kernel and drivers to fit your specific hardware needs.
- **Package Management:** ROS and its dependencies are natively distributed for Linux (specifically Ubuntu).

Since "Linux" is technically just the engine of an operating system, you need a full "car" to actually drive it. These different car models are called **Distributions** (or **Distros**).

### What is a Distro?
A Linux distribution (or distro) is a bundle of software that includes:

1. **The Kernel:** The "engine" (Linux kernel) that talks to the computer hardware.
2. **The Shell:** The command-line interface (like Bash).
3. **Desktop Environment:** The visual interface (icons, windows, taskbars).
4. **Package Manager:** The "App Store" (like `apt` or `dnf`) used to install software.

### Why are there so many?
Because Linux is open-source, anyone can take the kernel and build their own version. Different distros are designed for different purposes:

- **User-Friendly (Ubuntu, Mint):** Designed for beginners and developers. They come with everything pre-installed.
- **Security-Focused (Kali Linux):** Packed with tools for ethical hacking and privacy.
- **Server-Focused (Red Hat, Debian):** Built for stability and running the world's websites.
- **Minimalist (Arch Linux):** For experts who want to build their OS from scratch, piece by piece.

### The "Family Tree"

Most distros are actually descendants of three "ancestor" projects:

- **Debian:** The grandparent of **Ubuntu**.
- **Red Hat:** The base for many corporate and enterprise servers.
- **Arch:** The base for ultra-customizable "bleeding edge" systems.

> TIP: It's worth taking a quick look at the [Linux Family Tree](https://en.wikipedia.org/wiki/List_of_Linux_distributions) 
---

## 2. What was ROS 1?
The **Robot Operating System (ROS)** is "Middleware" i.e. software that helps different hardware parts (sensors, motors) talk to each other.

### The "Master" Architecture
ROS 1 relied on a central process called the **ROS Master**. 
- **The Problem:** If the Master crashed, the entire robot failed. 
- **The Network:** It was designed for a single robot on a stable, wired connection.
---

## 3. What is ROS 2?
ROS 2 is the modern successor to ROS 1, built to handle real-world challenges like unstable WiFi, multi-robot systems, and security.

### Key Features:
1.  **No Single Point of Failure:** There is no "Master." Nodes find each other via peer-to-peer discovery.
2.  **DDS (Data Distribution Service):** Uses industrial standards for high-speed, reliable data transfer.
3.  **Security:** Includes built-in support for encryption and authentication.
---

### Summary: The Main Differences

| Feature | ROS 1 | ROS 2 |
| :--- | :--- | :--- |
| **Central Manager** | ROS Master (Required) | None (Distributed) |
| **Communication** | Custom TCP/UDP | DDS (Industrial Standard) |
| **Reliability** | Best Effort | Quality of Service (QoS) |
| **Multi-Platform** | Linux Only | Linux, Windows, macOS, RTOS |

---

## 4. Why Ubuntu for ROS2?
While ROS 2 can technically run on Windows, macOS, or other versions of Linux, Ubuntu is the "Tier 1" platform.
- Native Development: The developers of ROS 2 (Open Robotics) write their code specifically for Ubuntu first.
- Synchronized Releases: Every major version of ROS 2 is designed to pair with a specific "Long Term Support" (LTS) version of Ubuntu.

To understand the relationship between ROS 2 and Ubuntu, it helps to look at how their versions are synchronized. Each ROS 2 release is built specifically for a "Long Term Support" (LTS) version of Ubuntu to ensure the robot's software remains stable for years.


### ROS 2 and Ubuntu Compatibility Table
Here is a breakdown of how the versions align and what they provide:

| ROS 2 Release | Ubuntu Version | Release Year | Support Status | Key Feature / Focus |
| --- | --- | --- | --- | --- |
| **Foxy Fitzroy** | 20.04 (Focal) | 2020 | EOL (End of Life) | First major stable ROS 2 LTS. |
| **Humble Hawksbill** | 22.04 (Jammy) | 2022 | Supported (LTS) | Most widely used in industry today. |
| **Iron Irwini** | 22.04 (Jammy) | 2023 | Short-term | Added improved hardware acceleration. |
| **Jazzy Jalisco** | **24.04 (Noble)** | **2024** | **Latest (LTS)** | **Our Course Focus:** Newest features. |
| **K-Turtle (Upcoming)** | 24.04 (Expected) | 2025 | Developing | Upcoming interim release. |


>  If you try to install **Jazzy** on Ubuntu **22.04**, it will fail because Jazzy requires specific versions of Python (3.12+) and C++ compilers that only come standard on Ubuntu **24.04**.


---

## 5. What is a Virtual Machine?
A **Virtual Machine (VM)** is a "software computer" that runs inside your physical computer. It acts like a completely separate device with its own operating system.

### How it works

- **Host:** Your physical computer.
- **Guest:** The Virtual Machine.
- **Hypervisor:** The software that bridges them, carving out a slice of your RAM and CPU for the VM to use. We will be using **VMware** in this class.

### Why use one?

- **Safety:** Run risky files in an isolated environment; if the VM crashes, your main computer is safe.
- **Compatibility:** Run Windows apps on a Mac, or Linux on Windows.
- **Snapshots:** Save the VM's state and "undo" any mistakes instantly.

## 6. Git: Collaborative Version Control
Git is the industry-standard version control system that allows developers to track changes in their code over time. 

Think of it as a "save game" system for your programming projects: it allows you to take "snapshots" (called commits) of your work, so if you break your robot's code during a lab, you can instantly revert back to a version that worked. 

Beyond just a safety net, Git enables multiple people to work on the same codebase simultaneously through "branching," where different features can be developed in isolation and later merged together seamlessly.

> Fun Fact: Linus Torvalds created Git in 2005 because he was frustrated with existing tools while developing the Linux kernel. Today, those two technologies form the "backbone" of modern robotics (and the internet), allowing millions of developers to collaborate on complex systems like ROS 2.

## 7. Mobile Robotics: The Hardware
A mobile robot is a system that can move through and interact with its environment.
We are using a [TurtleBot 4 Lite](https://en.wikipedia.org/wiki/TurtleBot) in this course. Here are some of its major hardware components:
- **iRobot Create 3 Base:** Handles the movement (kinematics) and basic safety (cliff sensors).
- **OAK-D Lite:** A spatial AI camera that provides 3D depth data (Point Clouds).
- **RPLidar A1:** A 2D laser scanner that "sees" obstacles in a 360° circle.
- **Differential Drive System:** Two powered wheels and one caster wheel. This allows the robot to rotate in place (zero-turn radius).
- **IMU (Inertial Measurement Unit):** An onboard 6-axis sensor (accelerometer and gyroscope) that helps the robot track its orientation and tilt.
- **Internal Odometry:** Optical encoders on the motors that count wheel rotations to estimate distance traveled.
- **Safety Bumpers & Cliff Sensors:** Tactile sensors on the front and infrared sensors on the bottom to prevent the robot from hitting walls or falling down stairs.
- **Raspberry Pi 4 Model B:** The TurtleBot 4 Lite is powered by a Raspberry Pi 4. This is where the Ubuntu 24.04 OS and your ROS 2 nodes actually run.

---

### Learn More:
* [Official Ubuntu Tutorials](https://ubuntu.com/tutorials/command-line-for-beginners)
* [The Linux Command Line (Free Book)](https://linuxcommand.org/tlcl.php)
* [Linux Foundation Training](https://training.linuxfoundation.org/)
* [Official ROS 2 Documentation](https://docs.ros.org/en/jazzy/index.html)
* [ROS 2 Tutorials (Beginner to Advanced)](https://docs.ros.org/en/jazzy/Tutorials.html)
* [TurtleBot 4 User Manual](https://turtlebot4.github.io/turtlebot4-user-manual/)


