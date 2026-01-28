# RAS 598: Mobile Robotics @ ASU

## Course Description
This graduate course explores the fundamental algorithms and mathematical frameworks that enable autonomous mobile robots to perceive, reason, and act in complex environments. The curriculum bridges theory with practical implementation, covering kinematics, sensor fusion, Bayesian estimation, probabilistic localization, mapping, and motion planning. A strong emphasis is placed on hands-on experience through programming labs and a final team project using the Robot Operating System (ROS).

## Timing
M/W 3:00 PM - 4:15 PM

## Location
Poly - ISTB12, Room 212

## Important Dates
First Day of Classes: January 12, 2026
Last Day of Classes: Apr 29, 2026
Final Project Due: May 6, 2026

# Course Schedule
```warning
Subject to change based on class size, group formation, and lab availability
```

| Wk | Date | Day | Topic | Description | Lab Time | Labs and Assignments START | Labs and Assignments END |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Jan 12 | M | Introduction to Robotics | Syllabus review & course structure |  | Reading for Quiz 1 |  |
| 1 | Jan 14 | W | ROS Overview | ROS publishers/subscribers, services, params |  | Quiz 0 for Lab 0 | Reading for Quiz 1 |
| 1 | Jan 16 | F |  |  |  | Lab 1: Setup (VM and ROS Basics) |  |
| 2 | Jan 19 | M | NO CLASS | Holiday: Martin Luther King Jr. Day |  |  |  |
| 2 | Jan 21 | W | A Primer on Coordinate Frames | Coordinate Frames and Transformation matrices | 45 min |  |  |
| 3 | Jan 26 | M | Coordinate Frames in ROS | Quaternions and ROS TF | 40 min |  | Lab 1: Setup (VM and ROS Basics) |
| 3 | Jan 28 | W | A Primer on Kinematics and Dynamics | Overview of Kinematics, Dynamics and Control Systems |  | Lab 2: Simulation |  |
| 4 | Feb 2 | M | A Primer on Perception | Overview of Robot Sensors and Levels of Perception | 30 min |  |  |
| 4 | Feb 4 | W | Range Sensors | Line fitting (RANSAC, Hough Transform) | 20 min | Lab 3: Robot Setup | Lab 2: Simulation |
| 5 | Feb 9 | M | Robot Setup | Robot Setup | 75 min |  |  |
| 5 | Feb 11 | W | Image Processing | Color spaces, filters, edge detection | 20 min | Report: Form Groups and Initial Ideas |  |
| 6 | Feb 16 | M | Feature Matching | SIFT, ORB | 20 min | Lab 4: Sensor-Motor Loop | Lab 3: Robot Setup |
| 6 | Feb 18 | W | 3D Point Clouds and Filtering | Voxel grid, statistical outlier, pass-through filters | 20 min |  |  |
| 7 | Feb 23 | M | Segmentation and Registration | Euclidean Cluster Extraction & ICP |  | Assignment 1: Perception | Lab 4: Sensor-Motor Loop |
| 7 | Feb 25 | W | Local Planning I | Motion Planning, BUG Algorithms | 25 min |  | Report: Form Groups and Initial Ideas |
| 8 | Mar 2 | M | Local Planning II | Dynamic Window Approach (DWA), Potential Fields | 25 min |  |  |
| 8 | Mar 4 | W | Global Planning I | Map Representations & Occupancy Grids | 5 min | Report: Refine Ideas |  |
| 9 | Mar 9 | M | NO CLASS | Spring Break |  |  |  |
| 9 | Mar 11 | W | NO CLASS | Spring Break |  |  |  |
| 10 | Mar 16 | M | Global Planning II | Greedy, A*, Dijkstra’s Algorithms | 5 min |  |  |
| 10 | Mar 18 | W | Sampling-based Planners | RRT and PRM |  | Assignment 2: Motion Planning | Assignment 1: Perception |
| 11 | Mar 23 | M | Bayes Filter | Uncertainty and Gaussian noise models | 15 min |  |  |
| 11 | Mar 25 | W | Bayes Filter (Cont.) | Covariance propagation | 15 min |  |  |
| 12 | Mar 30 | M | Sensor Model | Beam model and likelihood field model |  |  |  |
| 12 | Apr 1 | W | Motion Model | Odometry and velocity models |  |  | Report: Refine Ideas |
| 13 | Apr 6 | M | Kalman Filter | Linear Kalman filter | 20 min | Assignment 3: Bayes Filter | Assignment 2: Motion Planning |
| 13 | Apr 8 | W | Extended Kalman Filter | EKF localization & Nonlinear state estimation | 20 min |  |  |
| 14 | Apr 13 | M | Particle Filter | Non-parametric distributions & importance sampling |  |  |  |
| 14 | Apr 15 | W | Brief Introduction to SLAM | Introduction to SLAM |  | Report: Finalize Ideas |  |
| 15 | Apr 20 | M | Ethics in AI | Theoretical concepts and case studies |  |  |  |
| 15 | Apr 22 | W | Ethics in AI | Responsibilities and AI ethics |  |  |  |
| 16 | Apr 27 | M | Hackathon | Initial project setup |  |  |  |
| 16 | Apr 29 | W | Final Presentations | Demo and Poster Presentations |  |  | Assignment 3: Bayes Filter |
| 17 | May 4 | M |  |  |  |  |  |
| 17 | May 6 | W |  |  |  |  | Report: Finalize Ideas |

---

# Grading Policy
## Course Policy: Mastery & Iterative Engineering

Robotics is an inherently iterative discipline. High-performance systems are rarely "correct" on the first attempt; they are the result of rigorous testing, failure analysis, and refinement. To mirror professional engineering workflows, this course employs a **Mastery-Based Redemption Policy** that rewards deep debugging and technical persistence.

### 1. The Mastery Token System

Each student is allotted **2 Mastery Tokens** for the semester.

- **Purpose:** One token allows you to resubmit any lab or programming assignment (excluding the Final Project) to improve your grade.
- **The Feedback Loop:** You may only trigger a resubmission **after** you have received your initial grade and feedback from the teaching staff.
- **The Window:** Once your grade is posted, you have exactly **seven (7) calendar days** to submit your revised work.
- **Eligibility:** Tokens may only be applied to assignments that received an initial "Good Faith Effort" (defined as a submission that attempts to address the problem).

### 2. Late Submission Policy
To ensure the course moves at a professional pace, late submissions incur significant flat penalties. **Mastery Tokens cannot be used to waive late penalties.**

- **Up to 24 Hours Late:** -15% flat penalty.
- **24 to 48 Hours Late:** -30% flat penalty.
- **Beyond 48 Hours:** Submissions are not accepted for credit.

### 3. Grading & Point Recovery
We use a **50% Recovery Model**. This allows you to earn back half of the points between your initial effective grade (after any late penalties) and a perfect score ().

**The Formula:**
$$Final\ Grade = Initial\ Grade + \frac{100 - Initial\ Grade}{2}$$

### 4. Resubmission Requirements
A resubmission will not be graded unless it includes the following three components in the repository:

- **A. Technical Reflection:** A brief document answering:
  1. **Root Cause:** What specifically caused the failure in the previous version? (e.g., sensor noise, PID windup, or logic race condition).
  2. **The Delta:** What specific logic, parameters, or hardware configurations were changed?
  3. **Validation:** How did you verify the new solution is robust?
- **B. Git Diff / Pull Request:** For any submissions with code, a GitHub link to a diff highlighting the specific lines of code changed between the original and the new version.
- **C. Video Demonstration:** For any hardware-based lab, an unedited 20–60 second video demonstrating the robot’s successful performance is mandatory.

---

### Case Study Examples

| Scenario | Raw Score | Timing | Effective Initial | Resubmission? | Final Grade |
| --- | --- | --- | --- | --- | --- |
| **The Precise Fix** | 70% | On Time | 70% | **Yes** | **85%** |
| **The Late Penalty** | 90% | 12h Late | 75% | **No** | **75%** |
| **Late + Mastery** | 90% | 12h Late | 75% | **Yes** | **87.5%** |
| **Hardware Crisis** | 80% | 40h Late | 50% | **Yes** | **75%** |

**Note on Late + Mastery:** If you submit 1 day late and get a 90%, your grade is a 75%. If you then use a token to resubmit and get 100% on the fix, your final grade is.

### 5. Logistics
To use a token, fill out the **Regrade Request Form** on the Canvas. Include the link to your revised GitHub repository and the video proof.

---