---
layout: default
title: Robot Localization
parent: Assignments
sort: 3
---

```danger
## Under Construction
This guide is currently being updated for the <strong>Fall 2025</strong> semester. Please note that some screenshots or terminal commands may change before the final release.
```

# Assignment 3: Robot Localization

In this assignment, you will move from deterministic planning to **Probabilistic Robotics**. You will implement a **3D Discrete Bayes Filter** (Histogram Filter) to estimate the pose of a robot in a known environment using noisy odometry and landmark sightings. Your task is to maintain a probability distribution (belief) that accurately tracks the robot's ground truth position.

> Note: The term **Histogram Filter** comes from the way we represent probability. Instead of using a smooth mathematical formula (like the bell curve in a Kalman Filter), we "decompose" the world into discrete bins or cells. 

* Each cell acts like a bin in a histogram, holding a value representing the probability that the robot is inside that specific volume of space.
* Because we use a grid of boxes, the probability distribution looks like a 3D bar chart (a histogram) where "tall" bars indicate the most likely locations.

## 1. Learning Objectives
* Implement a **3D Histogram Filter** over state space $$(x, y, \theta)$$.
* Model robot motion using the Turn-Go-Turn **Odometry Model**.
* Implement a **Likelihood Field** for range-and-bearing landmark measurements.
* Visualize high-dimensional probability distributions and trajectories in **RViz**.

---

## 2. Landmark-Based Maps

Unlike grid-based maps that represent every pixel, **Landmark Maps** represent the world as a sparse collection of identifiable points. For this assignment, the cave environment is populated with several static "Fiducials." These landmarks are your primary source of truth for correcting the drift accumulated by the robot's odometry.

### 2.1. Map Database
The following landmarks have been identified and placed within the Stage simulator world. You must use these exact global coordinates $$(l_x, l_y)$$ when calculating the **Expected Range** and **Expected Bearing** during your measurement update.

| Landmark ID | X-Coordinate  | Y-Coordinate  | Description |
| :--- | :--- | :--- | :--- |
| **10** | $$-5.00$$ | $$-5.00$$ | Near the starting area. |
| **20** | $$-5.00$$ | $$1.00$$ | Center-left corridor. |
| **30** | $$-2.00$$ | $$7.20$$ | Top-left alcove. |
| **40** | $$-1.00$$ | $$3.10$$ | Central passage. |
| **50** | $$-1.00$$ | $$-1.00$$ | Near the central junction. |



### 2.2. Measurement Geometry
When the robot's sensor detects a landmark, it provides a measurement $$z = [r, b]$$, where:
* **Range ($$r$$):** The Euclidean distance from the robot's center to the landmark.
* **Bearing ($$b$$):** The relative angle (in degrees) of the landmark compared to the robot's current heading. 

In your filter, you will compare these values against every cell $$(x, y, \theta)$$ in your belief grid. A cell is considered "highly probable" if a robot placed there would see the landmark at a similar distance and angle.

While the measurement model discussed in class accounts for four distinct sources of error, this assignment employs a simplified model that applies a Gaussian distribution around the ground truth value.

> **Note:** Because the world is symmetric, a single range measurement creates a "ring" of probability. It is only by incorporating the **Bearing** and the robot's hypothetical orientation **$$\theta$$** that you can collapse this ring into a single estimated pose over multiple measurement steps.

---

### 2.3. Sensor Observations
When the robot’s fiducial sensor detects a landmark in the Stage simulator, a red dotted line is drawn in Stage connecting the robot to the landmark. If a landmark is behind a wall in Stage or not in the sensor view (the blue shaded region), the robot won't see the landmark. The robot has the capability to see multiple landmarks.

The `stage_ros2` node emits a `/fiducials` message whenever a landmark is within the robot's field of view and range. It provides the id and location of each landmark from the robot's perspective. Compare it with the true location of the landmark which is stored in the variable `self.landmarks` in the boilder plate code.


> Note: To find the message type, you may use `ros2 topic info /fiducials`, and to find the message structure, use `ros2 interface show <message_type>`.

## 3. System Architecture

### 3.1 The 3D State Space
Unlike previous assignments where the robot "knew" its location, you must now track a **Belief Array** representing the probability that the robot is at a specific grid cell.
* **Grid Dimensions:** Your world is $$16\text{m} \times 16\text{m}$$ centered at $$(0,0)$$.
* **Spatial Resolution:** $$0.2\text{m}$$ per cell ($$80 \times 80$$ grid).
* **Angular Resolution:** $$a^\circ$$ per "slice" ($$b = \frac{360}{x}$$ orientation bins).
* **Memory Layout:** A 3D numpy array of shape `(80, 80, b)`.

### 3.2 Prediction Step (Motion Model)
When the robot moves, the probability distribution will "shift" and "spread." You will implement the **Odometry Motion Model**:
* **Turn-Go-Turn:** Decompose odometry into an initial rotation ($$\delta_{rot1}$$), a translation ($$\delta_{trans}$$), and a second rotation ($$\delta_{rot2}$$).
* **Shift Logic:** In a 3D grid, "Forward" is relative to the $$\theta$$ slice. A robot at $$0^\circ$$ shifts along the X-axis; a robot at $$90^\circ$$ shifts along the Y-axis.
    > Hint: You may use `np.roll` to shift probability. 
* **Diffusion:** Apply a small Gaussian Blur to the belief after every movement to represent increasing uncertainty (entropy) over time.

### 3.3 Update Step (Measurement Model)
When the robot sees a landmark (Fiducial Maker), the probability distribution will "sharpen".
* **Likelihood Calculation:** For every cell in your 3D grid, calculate the *expected* range and bearing . Compare these to the *actual* sensor readings using a Gaussian PDF.(Refer [Section 2.3(#22-measurement-geometry)])
* **Bayes Rule:** $$Bel(x_t) = \eta \cdot p(z_t \mid x_t) \cdot \overline{Bel}(x_t)$$.

---

## 4. Technical Constraints

### 4.1 Frame Transformation
The Stage world uses a global coordinate system where the center is $$(0,0)$$, meaning coordinates range from $$-8.0$$ to $$+8.0$$. However, numpy grid indices start at $$(0,0,0)$$ and typically follow a "Matrix" convention (Row, Column). You must implement a robust mapping:

* **Spatial Mapping:** * The world coordinate $$(-8.0, -8.0)$$ corresponds to the bottom-left of the environment. In your grid, `index [0, 0]` corresponds to the **Top-Left** $$(x=-8.0, y=8.0)$$.
* **Orientation Mapping:** * Grid index `0` in the $$\theta$$ dimension corresponds to $$0^\circ$$ (facing Positive X direction). Ensure all trigonometric functions (`sin`, `cos`) use **Radians**, while grid indexing uses **Integer Degrees** divided by your resolution.

### 4.2 Numerical Stability
* **Normalization:** After every Predict and Update step, you must re-normalize the belief so that $$\sum Bel = 1.0$$.
* **Zero-Handling:** Add a tiny epsilon ($$1e-9$$) before dividing to prevent `NaN` errors if the probability distribution collapses.

### 4.3 Gotchas

* **Noise Parameters**: To model uncertainty, apply the Gaussian function `scipy.ndimage.gaussian_filter`. The specific covariance, or $$\sigma$$ values, used in this operation must be carefully tuned to reflect the actual noise levels observed in your odometry and sensor models, ensuring that the filter's uncertainty growth aligns with your experimental results and error analysis.
* **Visualization:** Use the `viz/odom_path` topic to visualize the robot's estimated trajectory in RViz. Do **NOT** use the raw `/odom` topic for visualization, as it may not align with your grid's coordinate frame.
* **Ground Truth** The Ground Truth topic (`/ground_truth`) is provided strictly for performance analysis and path visualization. It **must not** be used as an input to your Bayes Filter calculations. The filter should only "see" what a real robot would: noisy odometry and landmark sightings.

---

## 5. Setup

We will be using the Stage simulator, a lightweight alternative to Gazebo. Follow the steps in this section to install/update the simulator and download the assignment 3 base files.

### 5.1. Stage Simulator
> Note: You should have stage_ros2 installed from Assignment 2. This is a different version. Please delete the stage_ros2 folder from your ros_ws workspace.

```
cd ~/ros_ws/src

# Remove old stage_ros2 repo
rm -rf stage_ros2

# Pull the repo with the new branch 'bayes'
git clone -b bayes https://github.com/ras-mobile-robotics/stage_ros2.git

# Build
cd ~/ros_ws
colcon build
```

### 5.2. Boiler Plate Code

> Note: You can change the boiler plate code in any manner, as long as it satisifies the deliverables.

```bash
cd ~/ros_ws/src
git clone https://github.com/ras-mobile-robotics/ras598_assignment_3.git
```

##### File Structure
- `bayes.py`: The launch file that launches **map_server** (used for visualizing the map in RViz) and the **grading scout**.
- `bayes.rviz`: An example RViz config.


### 5.3. Run Stage Simulator

```bash
QT_QPA_PLATFORM=wayland ros2 launch stage_ros2 demo.launch.py world:=cave use_stamped_velocity:=false
```

### 5.4. Run RViz for Visualization

```bash
rviz2 -d ~/ros_ws/src/ras598_assignment_3/bayes.rviz
```

## 6. Code Structure and Submission Requirements
To facilitate automated grading and performance comparison, all submissions must adhere to the following naming conventions and interface standards.

## 6.1. File Naming and Launching
The package must be named **ras598_assignment_3**. You are required to provide a valid launch file **bayes_launch** that initializes the complete Bayes filter stack i.e. RViz (with the specified config), the simulator  and the Bayes filter node via the command:

```bash
ros2 launch ras598_assignment_2 bayes_launch.py
```

## 6.2. Visualization Standards
Since 3D arrays cannot be viewed directly, you must project your belief for RViz. To represent a 3D belief grid (X, Y, and Theta) as a 2D costmap in ROS, the function performs a marginalization of the orientation axis.

| Type | Topic Name | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Publisher** | `viz/belief_costmap` | `OccupancyGrid` | The 3D belief summed over the $$\theta$$ axis and normalized to $$[0, 100]$$. |
| **Publisher** | `viz/landmarks` | `MarkerArray` | 3D Cylinders showing landmark positions parsed from the Stage configuration world file. |
| **Publisher** | `viz/gt_path` | `Path` | A Green line showing the true trajectory from `/ground_truth`. |
| **Publisher** | `viz/odom_path` | `Path` | A Red line showing the raw, uncorrected odometry. |


---

## 7. Evaluation Criteria & Rubric

### 7.1 Technical Implementation (15 Points)
| Criteria | Description |
| :--- | :--- |
| **Coordinate Mapping (2 pts)** | Correct implementation of `real_to_grid` and `grid_to_real`. |
| **Motion Model (5 pts)** | The belief moves correctly with the robot's motion. If the robot moves up, the "cloud" of probability should move up as well. |
| **Measurement Model (5 pts)** | Landmark sightings correctly collapse the "cloud" into a distinct or multi-modal peaks. |
| **Convergence (2 pts)** | After seeing 2-3 landmarks, the peak of your belief should be within $$0.5\text{m}$$ of the Ground Truth. |
| **Explanation (1 pt)** | (Max 200 words) Explain why the probability distribution is ring-ish shaped, ellipse or some other shape with screenshots of the respective probability distribution. |

### 7.2 Viva Voce (Penalty Only)

The Viva Voce is a **mandatory** defense of your code. Failure to explain the following will result in point deductions:
* **Probabilistic Logic:** Explain how the bayes filter logic is transformed into code.
* **Scenario:** Explain various scenarios discussed in class with respect to your implementation (kidnapped robot, uniform initial belief, grid resolution trade offs, etc)

> **Note on Authenticity**: Whether you wrote every line from scratch or used an LLM as a collaborator, you are expected to understand your script. While forgetting the specific syntax of a rare NumPy function is excusable, failing to explain a logic block or a specific function call that appears multiple times in your code suggests a lack of understanding and will be penalized.

---