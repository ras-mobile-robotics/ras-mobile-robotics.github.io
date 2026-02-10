---
layout: default
title: Bare Metal ROS2 Setup
parent: Tutorials
sort: 8
---
## Bare Metal Setup: Ubuntu 24.04 & ROS2 Jazzy

This guide will help you configure your bare metal Ubuntu 24.04 machine to communicate with your assigned robot over the class network.

> **Pre-requisites:**
> If you haven't installed Ubuntu 24.04 or ROS2 Jazzy yet, please follow the tutorial for [Installing Ubuntu 24.04 and ROS2 Jazzy](../tutorials/ros_install)

### 1. Configure ROS2 Environment

To prevent cross-talk between different student robots, we use unique **Domain IDs** and a **FastDDS Discovery Server**.

First, clone the setup repository to your home directory:

```bash
cd ~
git clone https://github.com/ras-mobile-robotics/vm_setup.git
```

### 2. Run the Setup Script

Navigate to the directory, ensure the script is executable, and run it. This script will automate your network and environment variables.

```bash
cd ~/vm_setup
git pull
chmod +x bare_metal_setup.sh
./bare_metal_setup.sh
```

---