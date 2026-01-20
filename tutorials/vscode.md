---
layout: default
title: "VS Code & SSH Setup"
parent: Tutorials
sort: 6
---

# VS Code & SSH Setup Guide

This guide will walk you through setting connecting to a **Remote Virtual Machine (VM)** via SSH (to the robot), and installing the essential extensions for ROS 2 and Python development.

---

## 1. Install the Remote - SSH Extension
VS Code does not support SSH out of the box; you need the official extension from Microsoft.

1.  Open VS Code.
2.  Click on the **Extensions** icon on the left-hand sidebar (the four squares) or press `Ctrl+Shift+X`.
3.  Search for **"Remote - SSH"**.
4.  Click **Install** on the version published by Microsoft.

---

## 2. Connect to a remote machine
1.  In VS Code, press `F1` or `Ctrl+Shift+P` to open the **Command Palette**.
2.  Type `Remote-SSH: Connect to Host...` and select it.
3.  Select **Add New SSH Host**.
4.  Enter: `ssh username@remote_machine_ip`
5.  Select the SSH configuration file to update (usually the first option in the list).
6.  When prompted, click **Connect** in the bottom-right notification.
7.  **Important:** Once connected, the bottom-left corner should display `SSH: [IP_ADDRESS]`.

---

## 3. Recommended Extensions

> **Note:** When working over SSH, you must install extensions **on the Remote VM**. In the Extensions view, look for the button that says **"Install on SSH: [IP]"**.

### ROS 2 & Robotics
- **Robot Developer Extension for URDF/Xacro**: Essential for visualizing and editing your TurtleBot 4 robot models.
- **YAML (Red Hat)**: Provides syntax checking for ROS 2 launch files and parameter files.

### Python & Quality Control
- **Python (Microsoft)**: The base extension for all Python development.
- **Pylance**: High-performance "IntelliSense" (autocompletion and type checking).
- **Ruff**: The fastest linter and formatter available in 2026.
- **Error Lens**: Highlights errors and warnings directly in the line of code where they occur, so you don't have to check the "Problems" tab.

### General Productivity
- **GitLens**: Helps you see who changed a line of code and provides a deep history of your repository.
- **Peacock**: Changes the color of your VS Code window border. This is very helpful when you have multiple SSH windows open (e.g., one for the VM, one for the TurtleBot).
- **Path Intellisense**: Autocompletes filenames when you are typing paths in your code.

---

## 4. Format on Save

To ensure your code always follows professional standards, enable **Format on Save**.

1.  Open Settings (`Ctrl + ,`).
2.  Search for **"Format on Save"** and check the box.
3.  Search for **"Default Formatter"** and set it to **Ruff** for Python files.

---

## 5. Powering OFF
To prevent data loss, always shut down the software before cutting hardware power.

1.  **Shutdown the Pi/VM:** In the VS Code terminal, run:
    ```bash
    sudo shutdown now
    ```
2.  **Close Connection:** Once the VM shuts down, VS Code will notify you that the connection was lost. You can then safely close the window.

---

## Troubleshooting & Links
- [Official ROS 2 VS Code Setup](https://marketplace.visualstudio.com/items?itemName=Ranch-Hand-Robotics.rde-ros-2)
- [Ruff Documentation](https://docs.astral.sh/ruff/)