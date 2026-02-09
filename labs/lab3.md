---
layout: default
title: "Robot Setup, Basic Motion and Visualizatoin"
sort: 3
---

# Lab 3: Robot Setup, Basic Motion, and Visualization

In this lab, you will configure your Ubuntu 24.04 VM to communicate with a physical **TurtleBot 4** via a **Discovery Server**, visualize real-time sensor data in **RViz2**, and write a **Python node** to navigate the robot in a specific geometric path.

---

## 1. Collaboration & Submission Policy

Students work in groups of three to share one physical robot. You are encouraged to collaborate on setup and troubleshooting. However, **each student must submit their own unique motion script**. To ensure individual work, each member of your trio must choose a different shape from [Task 4]((../labs/lab2#7-task-4-robot-motion-node)) (Rectangle, Triangle, or Pentagon).

## 2. Power Management

The TurtleBot 4 uses two systems: the **Raspberry Pi 4** and the **Create® 3 Base**. Correct shutdown is essential to prevent SD card corruption.

### 2.1. Startup
1. Place the robot on the charger dock.
2. Wait ~3 minutes for the robot to initialize:
   1. First, you will hear a chime <audio controls src="{{ '/assets/audio/cb_startup.wav' | relative_url }}"></audio> that indicates the Create Base is booted up
   2. Soon after, you will hear a second chime <audio controls src="{{ '/assets/audio/robot_ready.wav' | relative_url }}"></audio> while a **fast purple light** rotating in the ring. This indicates the RPi and Create 3 are communicating, and the Robot is ready to use!

### 2.2. Graceful Shutdown
> This procedure ensures a graceful shutdown of both the Raspberry Pi and the Create 3 base, preventing SD card corruption and hardware-level data loss.

1. Move the robot off the charger.
2. In your SSH session: `sudo shutdown -h now`.
3. Wait 30 seconds, then hold the **Power Button** (large ring button) for about 8 seconds until you hear a chime <audio controls src="{{ '/assets/audio/cb_shutdown.wav' | relative_url }}"></audio> and the LED turn off.

---

## 3. Environment Setup

### 3.1. ROS2 Environment Setup
To ensure you only communicate with your assigned robot, we use specific **Domain IDs** and a **FastDDS Discovery Server**.

Run the commands below to pull the latest changes and run the setup script:

```bash
cd ~/vm_setup
git pull
./setup_robot_env.sh
```
---

### 3.2. Setup Bridged Network Mode

Your VM must be in **Bridged Network** mode to act as a physical device on the local network.

### Verify Connection

Run this command in your VM terminal:

```bash
ip route get 8.8.8.8
```

The output should show a local IP address (e.g., `192.168.50.x`). If it shows `172.x.x.x` or `10.x.x.x`, you are in NAT mode and must switch to Bridged mode in your VM settings (VMware/VirtualBox) and restart.

---

## 4. Task 1: Remote Access and Hardware Check

### 4.1. Ping the Robot

```bash
ping turtlebot
```

> `turtlebot` is a hostname that translates to the IP address of your specific robot, configured by the `setup_robot_env.sh` script. It makes your life easier by not having to remember the IP Address of your robot.

### 4.2. SSH into the Robot

SSH (Secure Shell) is the industry-standard method for connecting to a remote computer (like your TurtleBot 4 or a VM) over a network. It encrypts all traffic to prevent eavesdropping.

It creates an encrypted tunnel to the robot. Once you authenticate, your local terminal "hands over" its interface to the remote machine. You are no longer typing on your laptop; your command prompt effectively "teleports" to the robot, and every command executes on the robot's processor.

```bash
ssh ubuntu@robot
```
> **Password:** `turtlebot`

You will see a warning: "The authenticity of host... can't be established." when you connect to a remote machine for the first time. Type **yes** and press Enter.

### 4.3. Check Nodes

Once logged in via SSH, verify the sensors are broadcasting:

```bash
ros2 node list
```

You should see nodes like `/motion_control`, `/oakd`, and `/rplidar_node`.

> If your VM env is setup correctly, you can also run the same command on your VM.

---

## 5. Task 2: Sensor Visualization (RViz2)
RViz2 (ROS Visualization) is a 3D visualizer that allows you to see what the (real or simulated) robot "sees." It takes abstract data, like sensor readings, camera feeds, and coordinate frames, and renders them into a 3D environment that humans can understand.

Read this [RViz2 User Guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) before proceeding further. 
> You do not need to install RViz2; it is pre-installed in your VM. 

Launch **RViz2** on your VM (locally, **not** via SSH) to visualize the robot's "eyes."
1. **Run RViz:** Type `rviz2` in your VM terminal.
2. **Global Options:** Set **Fixed Frame** to `rplidar_link`.
> *Note:* The `scan` messages use this frame ID. If you use `map` without SLAM running, the data will not appear.

1. **Add Displays:**
* **LaserScan:** Topic `/scan` (Set Reliability to **Best Effort**).
> The LIDAR does not spin when the turlebot is docked!
* **TF:** To see the robot's transform tree.
* **Image:** Topic `/color/preview/image` to see the OAK-D camera feed.

---

## 6. Task 3: Manual Control and Message Inspection

Before writing your script, you should manually drive the robot to observe how the `/cmd_vel` topic translates movement into data. In Lab 3, you did this with a simualted robot.

### 1. Run Teleop (VM Terminal)

Open a new terminal on your VM and run the keyboard teleop node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2. Echo the Velocity Topic (Second VM Terminal)

Open another terminal and "eavesdrop" on the commands being sent to the robot:

```bash
ros2 topic echo /cmd_vel
```

The teleop_twist_keyboard node is sending the appropriate commands to the topic /robot_<RID>/cmd_vel based on the key you pressed.

### 3. Observe the Patterns

Try the following movements and watch the terminal output:

* **Go Straight:** Notice that only `linear.x` has a value.
* **Turn in Place:** Notice that only `angular.z` has a value.
* **Curved Path:** Use the "diagonal" keys (like `u` or `o`). Observe that **both** `linear.x` and `angular.z` have non-zero values simultaneously.

> **Reflection:** When you write your Python node, you are simply automating these same numbers!

---

## 7. Task 4: Robot Motion Node

```warning
**Do NOT write your ROS 2 nodes directly on the robot.** Since you are sharing hardware and ROS 2 is distributed, you must program and run your nodes within packages on your **respective VMs**.
```

Each student must implement **one** of the following shapes. Coordinate with your team to ensure no duplicates.

| Shape | Required Logic | External Turn Angle |
| --- | --- | --- |
| **Choice A: Rectangle** | 4 sides (Length: 1.0m, Width: 0.5m) | 90° |
| **Choice B: Triangle** | 3 equal sides (Length: 0.8m) | 120° |
| **Choice C: Pentagon** | 5 equal sides (Length: 0.6m) | 72° |

### Starter Code Template (`robot_drive.py`)

> Below is a reference code. You are free to modify the structure or implementation as needed.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotDrive(Node):
    def __init__(self):
        super().__init__('robot_drive')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1) # Wait for publisher to register
        self.get_logger().info('Starting Motion Loop...')
        self.execute_shape()

    def move_forward(self, duration):
        msg = Twist()
        msg.linear.x = 0.2  # 0.2 m/s
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.stop()

    def turn_robot(self, duration):
        msg = Twist()
        msg.angular.z = 0.5 # rad/s
        self.publisher_.publish(msg)
        time.sleep(duration) 
        self.stop()

    def stop(self):
        self.publisher_.publish(Twist())
        time.sleep(0.5)

    def execute_shape(self):
        # TODO: Implement your choice (A, B, or C) here using loops
        pass

def main():
    rclpy.init()
    node = RobotDrive()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

## 8. Submission Instructions
1. **Your Script:** `robot_drive.py` (Include your shape choice in the comments).
2. **Video Submission:** A clear unedited video showing (in the order below):
* You running `hostname; check-ros` on your VM terminal in your laptop.
* The physical robot completing the shape on the floor.
* After the robot stopped, recording tour laptop screen showing **RViz** with live LiDAR data.
  
---
