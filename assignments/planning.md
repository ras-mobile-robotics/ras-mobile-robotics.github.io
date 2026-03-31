---
layout: default
title: Motion Planning
parent: Assignments
sort: 2
---

# Assignment 2: Motion Planning

In this assignment, you will transition from perception to action. You will implement a complete navigation stack in a 2D environment using the **Stage** simulator. Your task is to navigate a robot through a cave environment to a target while minimizing **Total Energy Consumption**.


## 1. Learning Objectives
* Implement **Global Path Planning** (A*) to find a collision-free path on an occupancy grid.
* Implement a **Local Planner/Controller** to execute the path.
* Optimize robot motion to minimize energy losses due to **Inertia** and **Friction**.

---

## 2. System Architecture

### 2.1 Occupancy Grid Mapping
The environment is a static cave defined by the image `cave_filled.png`. To perform A*, you must first represent the bitmap image as an **Occupancy Grid**.
* **Representation:** Each cell in your grid must represent a **0.2m x 0.2m** area of the world. 
* **Obstacle Inflation:** Since the robot has a physical width, you must "inflate" the black pixels (walls) in the image by a safety radius ($$r > 0.6m$$). This ensures that the *center* of the robot can follow a path without the *edges* of the robot clipping a wall.
* **Coordinate Mapping:** You must implement a transformation between **Pixel Coordinates** $$(u, v)$$ from the image and **World Coordinates**$$(x, y)$$ from the simulator. Note that the center of the Stage world is $$(0,0)$$, while the image origin is typically the top-left corner.

### 2.2 Global Planner
Your node must load the map, inflate obstacles to account for the robot's footprint, and run A* from the `start` to the `goal` coordinates. (You may use OpenCV to load the image and inflate the obstacles using the `dilate` function).

You must implement the A* algorithm from scratch (Do not use off-the-shelf navigation libraries) on the occupancy grid.

### 2.3 Local Planner
To execute the local plan, you will implement a **Turn-Go-Turn** controller. 
* **Control Interface:** Publish `geometry_msgs/Twist` commands to the `/cmd_vel` topic.
* **State Feedback:** You may use the `/ground_truth` topic to obtain the robot's precise pose ($x, y, yaw$) in the map.
* **Control Logic:** Your controller should align the robot's heading with the next target waypoint and then drive forward along the straight-line segment.

### 2.4 The Grading Scout (`grading_scout`)
The Scout is an external "Black Box" node that monitors your `/cmd_vel` and ground-truth position.
* **Task Service:** Call the `/get_task` service to receive `start` and `goal` coordinates. This **resets your energy counter**. The response is a string of the format "{start_x},{start_y},{goal_x},{goal_y}"
* **Energy Model:** Drain is calculated based on linear motion, angular rotation, and a "Startup Tax."
* **Feedback:** Monitor `/energy_consumed` for real-time scoring.

---

## 3. Technical Constraints

### 3.1 Motion Cost Model
The Scout calculates the **Total Energy** by accumulating a "Tick Cost" every $$100\text{ms}$$:


$$\text{Cost}_{\text{tick}} = \text{base\_drain} + (|v| \cdot \text{linear\_coeff}) + (|\omega| \cdot \text{angular\_coeff}) + \text{startup\_tax}$$


* **Base Drain:** At every tick, a constant amount of energy ($$\text{base\_drain}$$) is consumed to represent the robot's onboard electronics and idling costs, regardless of movement.
* **Motion Costs:** Energy is consumed proportionally to the robot's linear velocity ($$v$$) and angular velocity ($$\omega$$). 
    * **Turning ($$\omega$$)** is weighted more heavily than driving ($$v$$) to simulate the "scrubbing" friction and high torque required for a differential drive robot to pivot on the spot.
* **The Startup Tax:** The Scout applies a one-time **Startup Penalty** ($$\text{startup\_tax}$$) only at the exact moment the robot accelerates from a standstill ($$v \approx 0$$). 

> **Optimization Tip:** To achieve a low energy score, you must minimize the number of times your robot comes to a complete stop. Every time the robot stops and restarts to make a turn, you pay the $$\text{startup\_tax}$$ again.


### 3.2. Path Pruning

- To minimize energy, you must maintain **momentum**. A robot that stops at every grid cell to "re-plan" or "rotate-then-drive" will pay the `startup_tax` repeatedly and likely fail the assignment.
- Implement a pruner to skip intermediate grid nodes. If there is a clear **Line-of-Sight** between your current position and a future waypoint, navigate directly to the further point to keep your velocity $$v > 0$$.

### 3.3 Discrete State Control
To minimize **Angular Jitter** (micro-oscillations that drain battery via the `angular_coeff`), use a state-machine approach:
* **State 1 (Rotate):** If the heading error is above a threshold (e.g., $$0.15\text{ rad}$$), rotate **in place** until aligned.
* **State 2 (Drive):** Once aligned, set angular velocity to **exactly 0.0** and drive forward. 

---

## 4. Setup
We will be using the Stage simulator, a lightweight alternative to Gazebo. Follow the steps in this section to install the simulator and download the assignment 2 base files.

### 4.1. Install Dependencies

```bash
# Install dependencies
sudo apt update
sudo apt install libfltk1.3-dev ros-jazzy-ackermann-msgs -y

# Clone repos
cd ~/ros_ws/src
git clone https://github.com/ras-mobile-robotics/Stage.git
git clone https://github.com/ras-mobile-robotics/stage_ros2.git

# Init and Update rosdep
sudo rosdep init 
rosdep update

# Install ROS2 dependencies
rosdep install --from-paths ./Stage --ignore-src -r -y  # install dependencies for Stage
rosdep install --from-paths ./stage_ros2 --ignore-src -r -y  # install dependencies for stage_ros2

# Build Stage and Stage ROS2 Packages
cd ~/ros_ws
colcon build --symlink-install 
```
### 4.2. Clone Assignment 2 Files

```bash
cd ~/ros_ws/src
git clone https://github.com/ras-mobile-robotics/ras598_assignment_2.git
```

##### File Structure
- `planner_launch.py`: The launch file that launches **map_server** (used for visualizing the map in RViz) and the **grading scout**.
- `map.yaml`: The map configuration used in the **map_server** node.
- `cave_filled.png`: The bitmap (an image file) depicting the map. The bitmap is used in `map.yaml`
- `planning.rviz`: An example RViz config.

### 4.3. Running Instructions

#### 4.3.1 Run Stage Simulator

```bash
QT_QPA_PLATFORM=wayland ros2 launch stage_ros2 demo.launch.py world:=cave use_stamped_velocity:=false
```


#### 4.3.2 Run Launch File (Grading Scout and Map Server for RViz)
After creating your ROS2 pkg, you can run the launch file using using `ros2 launch ras598_assignment_2 planner_launch.py`.

You may also run it as below:

```bash
python3 planner_launch.py
```

#### 4.3.3 Run rviz for Visualization

```bash
rviz2 -d ~/ros_ws/src/ras598_assignment_2/planning.rviz
```


## 5. Code Structure and Submission Requirements
To facilitate automated grading and performance comparison, all submissions **must** adhere to the following naming conventions and interface standards.

```note
Depending on the location of the files, you may have to change the path of the `cave_filled.png` bitmap file in Line 1 of `map.yaml`, and `map_yaml_path` and `scout_script_path` in `planner_launch.py`.
```


### 5.1. File Naming & Launching
The package must be named `ras598_assignment_2`. You are required to provide a valid launch file `planner_launch` that initializes the complete navigation stack i.e. the **map_server, RViz (with the specified config), planner, grading_scout, and the simulator** via the command:

```bash
ros2 launch ras598_assignment_2 planner_launch.py
```

### 5.2. ROS 2 Topics and Services
Your node must interact with the following standardized topics. **Do not rename these**, or the Grading Scout will not be able to track your performance.

| Type | Topic / Service Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Subscriber** | `/energy_consumed` | `std_msgs/Float32` | Real-time feedback from the Grading Scout. |
| **Subscriber** | `/ground_truth` | `nav_msgs/Odometry` | [OPTINAL] Use this for exact $$(x, y, yaw)$$ coordinates. |
| **Publisher** | `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands ($$v, \omega$$). |
| **Publisher** | `/planner_markers` | `visualization_msgs/MarkerArray` | Visualize your path and waypoint goals in RViz. |
| **Service Client** | `/get_task` | `???` | Call this once at startup to receive your goal. |

### 5.3. Visualization Standards
Your `/planner_markers` must include:
1.  **A Green `LINE_STRIP`:** Representing the raw, unpruned A* path.
2.  **A Blue `LINE_STRIP`:** Representing your final **Pruned** path.
3.  **A Red `SPHERE`:** Representing the current goal coordinate for the local planner.


### 5.4. Planning Parameters
* **Grid Resolution:** _**Must be 0.2 meters**_ (The size of one A* grid cell).
* **Inflation Radius:** _**Must be $> 0.6$ meters**_. This is the safety buffer applied to all walls to account for the robot's physical footprint. You are encouraged to tune this value; a larger radius makes navigation safer but limits the paths available in narrow corridors.
* **Velocity Control:** If you notice your robot "skidding" past waypoints or bumping into obstacles due to momentum, you should reduce/cap your maximum linear and angular velocities.

> **Tip:** High speeds ($$v > 0.6 \text{ m/s}$$) in tight cave corridors often lead to overshooting, which triggers additional **Startup Penalties** and consumes more energy than a slower, steadier pace.


---

##  6. Evaluation Criteria & Rubric 
The instructor will run a **Head-to-Head Comparison**:
* **Energy Efficiency:** The final value of `/energy_consumed`.
* **Path Smoothness:** The number of "Startup Penalty" events recorded by the Scout.

A "Perfect" submission will show a **Blue Path** (Pruned) that has fewer vertices than the **Green Path** (Raw), typically resulting in a lower energy score due to fewer stops and turns.

### 6.1. Technical Implementation (15 Points)
* **A* Pathfinding (4 pts):** Successfully generates a collision-free path from start to goal.
* **Path Pruning (2 pts):** Correct implementation of Line-of-Sight (LOS) skipping or any kind of useful path pruning. The Blue path must have significantly fewer nodes than the Green path.
* **Path Execution (4 pts):** Robot correctly moves from the start to the goal without any collisions.
* **Competitive Energy (3 pts):** Your total `/energy_consumed` compared to the class baseline.
* **Standardization (2 pts):** All topic names, service calls, and marker colors match the specification.

### 6.2. Viva Voce (Penalty Only)
The Viva Voce is a **mandatory** defense of your code. Failure to explain the following will result in point deductions:
* **Coordinate Math:** Mapping pixels to world coordinates.
* **Algorithm Logic:** Explaining why the robot consumed a specific amount of energy.
* **Authenticity:** If you cannot explain your code, you will receive significant penalties for the entire assignment.

---
