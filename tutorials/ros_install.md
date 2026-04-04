---
layout: default
title: "Installing Ubuntu 24.04 and ROS2 Jazzy"
parent: Tutorials
sort: 10
---

```warning
This tutorial is for installing Ubuntu 24.04 and ROS 2 natively on your machine.

Proceed with this method **ONLY** if you have encountered persistent issues with the VM despite following the VM tutorial, debugging independently, and attending office hours.
```

# Installing Ubuntu 24.04 and ROS2 Jazzy

This tutorial provides a step-by-step guide to installing **Ubuntu 24.04 (Noble Numbat)** and setting up **ROS2 (Jazzy Jalisco)** using an **UNTESTED** automated script.

This guide is intended for users who wish to set up a native (bare metal) Linux environment or a fresh Virtual Machine. The script automates the installation of essential tools, ROS2, and workspace configurations.

## Prerequisites

* A USB drive (minimum 8GB) for Ubuntu installation.
* A stable internet connection.
* **Backup your data:** If installing on bare metal, ensure all important files are backed up as partitioning can lead to data loss.

---

## Step 1: Install Ubuntu 24.04 LTS

1. **Download the ISO:** Get the "Desktop" image from the [Official Ubuntu Website](https://ubuntu.com/download/desktop).
2. **Create a Bootable Drive:** Use a tool like [Etcher](https://www.balena.io/etcher/) or [Rufus](https://rufus.ie/) to flash the ISO onto your USB drive.
3. **Boot from USB:** Restart your computer and enter the BIOS/UEFI (usually F2, F10, or Del) to select the USB drive as the boot priority.
4. **Follow the Installer:**
* Select **"Install Ubuntu"**.
* Choose **"Interactive Installation"**.
* Select **"Extended selection"** (to ensure you have basic utilities).
* **Recommended:** Check the box "Install third-party software for graphics and Wi-Fi hardware."


5. **Restart:** Once finished, remove the USB drive and log into your new Ubuntu desktop.

---

## Step 2: Prepare the System

Before running the script, open a terminal (`Ctrl+Alt+T`) and ensure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install git curl -y
```

---

## Step 3: Run the Setup Script

The script provided by RAS automates the ROS2 Jazzy installation, configures environment variables, and installs necessary build tools (like Colcon and Rosdep).

1. **Download the script:**

    ```bash
    curl -O https://raw.githubusercontent.com/ras-mobile-robotics/vm_setup/main/deploy/setup_vanilla_ubuntu.sh
    ```


2. **Make it executable:**

    ```bash
    chmod +x setup_vanilla_ubuntu.sh
    ```

3. **Run the script:**

    ```bash
    ./setup_vanilla_ubuntu.sh
    ```


*Note: The script will prompt for your `sudo` password to install packages.*

---

## Step 4: Verify the Installation

After the script finishes, **restart your terminal** or run `source ~/.bashrc`. Then, verify ROS2 is working:

1. **Check ROS2 version:**

    ```bash
    echo $ROS_DISTRO
    ```

*Expected Output:* `jazzy`

2. **Run a Demo Talker:**

    ```bash
    ros2 run demo_nodes_cpp talker
    ```


3. **Run a Demo Listener (in a new terminal):**
    
    ```bash
    ros2 run demo_nodes_py listener

    ```

If the listener receives messages, your installation is successful!

---

## Troubleshooting

### 1. "Untested" Disclaimer

As noted, this script is considered "untested" for every specific hardware configuration. If the script fails, check which command caused the error. Most issues are resolved by running:

```bash
sudo apt --fix-broken install
```

### 2. Rosdep Initialization

If the script does not do it for you, you must initialize `rosdep` manually:

```bash
sudo rosdep init
rosdep update
```

### 3. Why Ubuntu 24.04?
Ubuntu 24.04 is the target OS for **ROS2 Jazzy**. Using older versions (like 22.04 or 20.04) with this script will likely result in "Package not found" errors.
