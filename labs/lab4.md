---
layout: default
title: "Reactive Navigation using ROS 2 Services and Launch Files"
sort: 4
---

# Lab 4: Reactive Navigation using ROS 2 Services and Launch Files

This lab serves as the bridge between basic movement and autonomy: your first sense-act loop. You will implement an **Automatic Emergency Braking (AEB) and Evasive Recovery** system. This pattern is foundational in mobile robotics, where a robot must transition from a "standby" state to an "active" state and autonomously react to environmental hazards.


## Learning Objectives:

* **Implement Closed-Loop Control:** Transition from simple open-loop timing to a sensor-driven feedback system.
* **Deploy ROS 2 Services:** Learn to create and call services to "arm" or toggle robot states.
* **Master Launch Files:** Utilize Python launch scripts to manage configurations, specifically focusing on parameters and topic remappings.
* **Develop Reactive Logic Loops:** Program the robot to process real-time LIDAR data to make autonomous navigation decisions.

---

## 0. Prerequisites

```note
Finish these tutorials before you come to the lab to work on the robot!
```

> Estimated Time: 50 mins

Before starting this lab, you must be familiar with the Request-Response paradigm in ROS 2. Complete the following official ROS 2 Jazzy tutorials:

### 0.1. Understanding Services:

Learn to interact with services via the terminal to debug your nodes using the tutorial
[here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html )

* **`ros2 service list`**: To see active services.
* **`ros2 service type <service_name>`**: To check the interface type.
* **`ros2 service call <service_name> <type> <arguments>`**: To manually trigger a service.
* **`ros2 interface show <type_name>`**: To inspect the request/response structure of a `.srv` file.

### 0.2. Writing a Simple Service and Client (Python)

Learn how to implement the service server logic within a Python node using the tutorial
[here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html).

### 0.3 Understanding Parameters:

Learn to interact with parameters using the tutorials
[Understanding ROS2 Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
and [Using Parameters In Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)

### 0.4. Custom Interfaces (msg/srv)

```note
This is an optional tutorial for this lab.
```

Understand how ROS 2 defines data structures for communication using the tutorial
[here](https://docs.ros.org/en/jazzy/Tutorials/Custom-ROS2-Interfaces.html).
While we are using the standard `std_srvs/srv/SetBool` for this lab, understanding how these interfaces are built is key to troubleshooting errors, especially for futre labs.

---

## 1. Collaboration and Submission Policy
Students work in groups of three to share one physical robot. You are encouraged to collaborate on setup and troubleshooting. However, **each student must submit their own unique script**. To ensure individual contribution, each member of your team must implement a unique logic-based behavior. To simplify testing in tight lab spaces, **do not include linear moves** in your evasive logic; focus strictly on orientation changes.

| Student | Trigger Condition | Evasive Action (Rotation Logic) |
| --- | --- | --- |
| **Choice A** | Obstacle < 0.5m (Front) | **The Randomizer:** Randomly rotate either Left or Right between 90° and 180° to find a new path. |
| **Choice B** | Obstacle < 0.5m (Left **AND** Right) | **The Retreat:** Triggered when the robot feels "squeezed." Perform a full 180° spin to exit the narrow space. |
| **Choice C** | Obstacle < 0.3m (Front) | **The Smart Pivot:** Perform a 90° turn toward the *clearest* side (whichever side, Left or Right, has the higher distance reading). |

```note
The Obstacles distances in "Trigger Condition" are measured from the LiDAR. You do not have to use TF for this lab!
```

---

## 2. Task 1: Create a Custom ROS 2 Service

The robot must remain stationary until it receives a "Go" signal via a ROS 2 Service.

1. Modify your node `reactive_navigator.py`.
2. Define a Service named `toggle_navigation` using the `std_srvs/srv/SetBool` type.
3. The node logic must check a boolean flag; the robot should **only** process LIDAR data and move if the service state is `True`.

---

## 3. Task 2: The Sensor-Action Loop (LIDAR)

Your node must subscribe to the scan topic.

* **The Challenge:** Raw LIDAR data contains a variable number of points. You must parse the `ranges` array.
* **Dynamic Indexing:** Do not use hard-coded array indices (e.g., `msg.ranges[0]`). You must calculate indices dynamically using the metadata (information that is not the range data) provided in the `LaserScan` message.

**Calculation Formula:**
Below is a reference formula to find the index for a specific angle (in radians):

$$index = \text{int}\left(\frac{\theta_{target} - \theta_{min}}{\theta_{increment}}\right)$$

**Logic Flow:**

1. If `navigation_enabled` is `True`:
   1. Calculate the indices for your assigned sectors (Front, Left, and/or Right).
   2. Filter out "Inf" or "NaN" values in the `ranges` array.
   3. If your specific **Trigger Condition** (see [Task 1 table](#1-collaboration-and-submission-policy)) is met: **Call your internal Rotation Maneuver function.**
2. Otherwise: 
   1. Maintain a slow forward velocity.

---

## 4. Task 3: ROS 2 Launch Files
To manage naming and parameters without hardcoding, you will create a Python Launch file named `turtlebot_bringup.launch.py`.

Your launch file must:

1. Start your `reactive_navigator` node.
2. Declare and pass a **ROS Parameter** called `safety_distance` (default: 0.5).

---

## 5. Task 4: Implementation Template

```note
Below is only a reference code. You are free to modify the structure or change the implementation as needed.
```

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import random

# CHOICE: X
class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')
        
        # 1. Setup Service
        self.srv = self.create_service(SetBool, 'toggle_navigation', self.toggle_callback)
        self.active = False

        # 2. Setup Pub/Sub
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # 3. Parameter
        self.declare_parameter('safety_distance', 0.5)

    def toggle_callback(self, request, response):
        # Service Callback
        self.active = request.data
        response.success = True
        response.message = f"Navigation set to {self.active}"
        return response

    def get_index_for_angle(self, msg, angle_deg):
        # Helper to convert degrees to msg index
        pass

    def scan_callback(self, msg):
        if not self.active:
            return

        # TODO: Calculate indices for your specific Member role
        pass

    def execute_rotation_maneuver(self):
        # TODO: Implement Member A, B, or C rotation logic here
        self.get_logger().info("Obstacle Detected! Executing rotation maneuver.")
        pass

    def drive_forward(self):
        msg = Twist()
        msg.linear.x = 0.05 
        self.publisher_.publish(msg)

```

---

## 6. Lab Check-off and Submission

### 6.1. Verification Steps

1. **Launch:** Run `ros2 launch my_package turtlebot_bringup.launch.py`.
2. **Verify Parameters:** Run `ros2 param list` to ensure the parameters are applied to your node.
3. **Trigger:** In a second terminal, call:
`ros2 service call /toggle_navigation std_srvs/srv/SetBool "{data: true}"`
4. **Test:** Use a static object to trigger the rotation. Verify the robot does not move forward during the maneuver.

### 6.2. Submission Requirements

1. **Code:** Your `reactive_navigator.py` (Include your choice in the comments) and `turtlebot_bringup.launch.py`.
2. **Video:**
* Terminal output of starting the launch file and calling the service.
* Robot starting to move and reacting to an obstacle with the correct rotation logic.
