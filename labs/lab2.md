---
layout: default
title: "Robot Setup"
sort: 2
---

```danger
## Under Construction
This guide is currently being updated for the <strong>Fall 2025</strong> semester. Please note that some screenshots or terminal commands may change before the final release.
```

This guide covers the configuration of the your **Host Computer** and a **TurtleBot 4** using the Discovery Server protocol. This ensures stable communication in a shared classroom network environment.

## Robot Configuration 

After the VM reboots, you must configure your specific TurtleBot 4 hardware via the physical screen.

### 1: Setup SSH Tutorial: Remote Access Made Simple

SSH (Secure Shell) is the industry-standard method for connecting to a remote computer (like your TurtleBot 4 or a VM) over a network. It encrypts all traffic to prevent eavesdropping.

---
> **Credentials**
> * **User:** `eva`
> * **Password:** `wall-E`

#### Basic Connection
From your terminal (Linux, macOS, or Windows PowerShell), use the following syntax:

```bash
ssh ubuntu@<ROBOT_IP>
```

> **First Time Connection?** > You will see a warning: "The authenticity of host... can't be established." 
> Type **yes** and press Enter. This adds the remote machine to your `~/.ssh/known_hosts` file.

---

#### Passwordless Login (SSH Keys)
Typing a password every time is slow. SSH Keys allow you to log in securely without a password using a "lock and key" system.

##### Step 1: Generate your Keys
On **your laptop**, run:
```bash
ssh-keygen -t ed25519
```
*Press Enter through all prompts to use the defaults.*

##### Step 2: Copy the Key to the Remote Machine
```bash
ssh-copy-id eva@<ROBOT_IP>
```

Once this is done, you can log in simply by typing `ssh eva@<ROBOT_IP>`.

---

#### Common SSH Tasks
| Action | Command Syntax |
| :--- | :--- |
| **Run a command & exit** | `ssh user@ip "ls -la"` |
| **Copy file TO remote** | `scp local_file.txt user@ip:/home/user/` |
| **Copy file FROM remote** | `scp user@ip:/path/to/file.txt ./` |
| **Edit remote file** | `ssh user@ip "nano config.yaml"` |

---


### 2: Turtlebot ROS2 Setup: Set Namespace and Discovery Server

Run the built-in ROS2 setup utility for turtlebot4:

```bash
turtlebot4-setup

```
Navigate through the robot menu and follow the interactive prompts carefully:

1. **ROS Setup** -> **Bash Setup**
* Set `ROBOT_NAMESPACE` to: `robot_XXXXXXXXXX` (Replace `X` with your assigned ID).

2. **ROS Setup** -> **Discovery Server**
* **Onboard Server - Server ID**: `X` (Must match your namespace ID).
* Select **Save**.

3. Return to the **Main Menu**.
4. Select **Apply Settings**.
* Confirm **YES**.
* **Wait:** The process takes 1–3 minutes to reconfigure the network layers.

5. **Exit** the utility.

Apply the changes to your current session and restart the ros2 daemon processes:

```bash
turtlebot4-source
turtlebot4-daemon-restart
```

> [!IMPORTANT]
> **Wait for the Chime:** Once the Create® 3 base config is loaded, the white spinning LED will stop. Wait until the LED ring settles on a **Solid White** state before proceeding.