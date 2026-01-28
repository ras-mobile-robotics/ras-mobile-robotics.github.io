---
layout: default
title: "Simulation"
sort: 2
---

# Lab 2: Coordinate Frames and Simulation
By the end of this lab, you will learn how to run a full 3D simulation on your own computer and how to interact with the ROS tf2 library using CLI tools and the Python API.

## Learning Objectives:
* Complete the official ROS 2 Jazzy TF2 Python tutorials.
* Run the TurtleBot 4 Gazebo simulation.
* Inspect the TurtleBot 4 tf tree using CLI tools.

```note
In this lab, you will be testing the VM's limits on your computer hardware. A lot of your effort might be focussed on the tuning and debugging involved in setting up your environment to run Task 3 (Gazebo simulator). As every student's computer has a different configuration, please use online resources, LLMs, class discussions, and office hours to help get the Gazebo simulator up and running.
```

## Task 1: Complete the ROS TF2 Tutorial
Before starting the (simulated) robot-specific task, you must complete the following official tutorials for **ROS 2 Jazzy**. These will help you understand how to use ROS tf2 and provide the boilerplate code you need for Part C.

1.  **[Learning about TF2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html):** Overview of `view_frames` and `tf2_echo`.
2.  **[Writing a Static Broadcaster (Python)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html):** How to define a frame that doesn't move.
3.  **[Writing a Listener (Python)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html):** Learn the `Buffer` and `TransformListener` pattern.

---

## Task 2: Run the Gazebo Simulation for Turtlebot4
In your ROS workspace, clone the simulation git repo:
```bash
git clone https://github.com/ras-mobile-robotics/ras598_sim.git
```

Build the ROS workspace and launch the TurtleBot 4 simulation:

```bash
ros2 launch ras598_sim turtlebot4_gz.launch.py
```

Wait 1-3 minutes until the log output in the terminal stabilize and you see the message below:

```bash
============================================================
       SIMULATION READY FOR STUDENTS
       The robot is undocked and ready for commands.
============================================================
```

**Note:** Due to race conditions or specific hardware limitations, the robot might not automatically undock. If after 1-3 minutes the terminal output is stable and not changing, run the command below to force an undock:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

It should reply with the message "*Goal finished with status: SUCCEEDED*" and the simulated robot should be undocked.

```note
If you had issues with runnning the simulation, the simulation being slow, or seeing a low of warning messages, checkout the VM guide for [Enable 3D Hardware Acceleration](../tutorials/vm#enable-3d-hardware-acceleration).
```

### Task 3: Robot Teleoperation
Run the **teleoperation node**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Move the robot around to verify it is working.


### Task 4: The Visual TF Tree

Run the following command to generate a PDF of the robot's hierarchy:

```bash
ros2 run tf2_tools view_frames
```

This generates a PDF of the entire tf tree. Take a look at it! If you are not sure how to open the file, this is a good opportunity to explore your VM interaction skills.

### Task 5: Measuring the Offset

The camera is located away from the Center of Mass (CoM) of the robot. Your goal is to find this offset.

In the frames list from Task 2, identify the frame name for the OAK-D Lite camera. It will likely contain the keywords `oakd` and `link` (ROS terminology usually combines these terms). Pick one of these frames for the command below.

Use the CLI to find exactly what that offset is:

```bash
ros2 run tf2_ros tf2_echo base_link <YOUR_CAMERA_FRAME_NAME>
```

---

## Deliverables

In Canvas, submit a short screen recording (~10 secs) for each of the following:
1. The turtles moving from [Task 1](../labs/lab2#task-1-complete-the-ros-tf2-tutorial).
2. Turtlebot moving around in Gazebo from [Task 3](../labs/lab2#task-3-robot-teleoperation).
3. tf tree and the terminal window printing the offest from [Task 5](../labs/lab2#task-5-measuring-the-offset).