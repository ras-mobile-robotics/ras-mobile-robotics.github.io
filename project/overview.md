---
layout: default
title: Overview
parent: Project
sort: 1
---

## Course Project Guide: Autonomous Mobile Robotics

The course project is designed to transition you from executing guided lab exercises to engineering autonomous systems capable of operating in complex environments. This requires the strategic integration of various systems: perception, state estimation, and path planning within the ROS 2 framework. You are expected to demonstrate graduate-level rigor by developing custom algorithmic modules from first principles while effectively combining established software packages to build a robust, end-to-end robotic system.

---

## 1. Project Proposal

## 1.1. Restrictions & Guidelines

To ensure the project meets the criteria for graduate-level Mobile Robotics, the following constraints apply:

### The "Mobile" Constraint

* **Mandatory Locomotion:** The system must actively navigate an environment. Stationary systems, such as pick-and-place arms on fixed pedestals, are not acceptable.
* **Mobile Manipulation:** If an arm is utilized, the project must emphasize "Base-to-Target" coordination (e.g., navigating to a specific location before performing a manipulation task).
> Note: The course does not have sanctioned arms available; however, procurement of hardware may be discussed on a case-by-case basis.


* **Autonomous Decision Making:** The system must operate without human intervention. Teleoperation (remote control) projects will be rejected.

### 1.2. Technical Checklist for Proposal Approval

* **Defined Workspace:** Specify if the environment is static, dynamic, indoor, or outdoor.
* **State Estimation:** Define the methodology for tracking the robot's pose (e.g., EKF, UKF, Particle Filter).
* **Perception Stack:** Identify the specific sensor data (LiDAR, RGB-D, IMU) being processed.
* **Planning Algorithm:** Specify the path planning logic (e.g., RRT*, A*, or Vector Field Histograms).
* **Custom Component:** Identify which specific ROS 2 node or module will be written from scratch (mathematical implementation).
* **Safety Intent:** Define the fail-safe behavior for hardware and software malfunctions.

---

## 2. Project Examples

### 2.1. The "Bad" (Insufficient Complexity or Lacking Mobility)

* **The Stationary Sort:** An arm sorting blocks by color on a fixed table.
  * *Shortcoming:* Lacks locomotion, localization, and mapping.


* **The Follow-the-Line Bot:** A robot following a high-contrast floor line.
  * *Shortcoming:* This is undergraduate-level control. It lacks probabilistic estimation and complex planning.


* **The Standard Nav Stack:** Deploying the default TurtleBot4 navigation package in a simple environment.
  * *Shortcoming:* This is a configuration task. Graduate projects require custom algorithmic depth.

* **The LLM Wrapper**: A robot that sends a prompt to an LLM (e.g., GPT-4) and simply receives a pre-defined command string like "Turn Left."
  * *Shortcoming*: This treats robotics as a text-processing problem. It bypasses the fundamental challenges of real-time state estimation, sensor noise handling, and closed-loop control.

### 2.2. The "Good" (Graduate Level & Mobile)

* **The Semantic Courier:** A robot that identifies objects (e.g., a "red chair") via Computer Vision and navigates to them in unmapped, cluttered spaces.
  * *Technical Depth:* CV Integration + SLAM + Dynamic Path Re-planning.


* **The Active Exploration Bot:** A robot that autonomously maps an unknown building by calculating "Frontiers" and selecting the most informative path.
  * *Technical Depth:* Information-gain mathematics + Occupancy Grid management + Frontier logic.


* **The Mobile Bartender:** A TurtleBot with a 4-DOF arm navigating through dynamic human crowds to deliver items.
  * *Technical Depth:* Dynamic Obstacle Avoidance (DWA tuning or Velocity Obstacles) + Mobile Manipulation.

* **The Common Sense Assistant**: A robot that receives high-level, ambiguous natural language commands (e.g., "Help me clean this coffee spill without waking the baby") and translates them into a sequence of constrained engineered tasks.
  * *Technical Depth*: Large Language Model (LLM) reasoning as a Task Planner + Custom Semantic Costmaps + Constraint-based Local Planning.

---

## 3. Project Milestone & Grade Structure
It follows the ["V-model" of engineering](https://en.wikipedia.org/wiki/V-model): starting with heavy requirements and design, moving into integration, and finishing with a deep-dive into the technical "guts" and performance.

| Milestone | Course Grade Weight | Doc/Tech Split | Focus |
| --- | --- | --- | --- |
| **Milestone 1: Proposal** | **5%** | **100% / 0%** | **Design & Intent:** Setting the mission, architecture, and safety protocols. The "contract" of what will be built. |
| **Milestone 2: Proof** | **10%** | **60% / 40%** | **Theory & Alpha:** Formal math/model and proof of progress in the code. |
| **Milestone 3: Final** | **15%** | **30% / 70%** | **Code & Analysis:** Code Documentation, Ethical Analysis, and performance analysis. |

### 3.1. Why this split?

1. **Milestone 1 (100% Doc):** At this stage, teams are often still fighting with design ideas, and workspace setup. Grading 100% on the **Proposal and Architecture** ensures you have a solid plan before they start coding.  It rewards clear thinking over messy, early-stage scripts.
2. **Milestone 2 (60/40 Split):** This is the transition. After incorporating instructor feedback, teams must explain the **Kinematics** model (Doc) while proving their code can handle **Noise and Robustness** (Tech). The documentation still carries weight because you are still adaptiving and defining the "How".
3. **Milestone 3 (30/70 Split):** The documentation becomes a "Scientific Dossier" (Benchmarking and Ethics), but the heavy lifting is the **System Integration and Custom Algorithm implementation.** The 70% Technical weight allows you to perform the **Git Audit** and verify that the "Algorithmic Factor" was actually achieved in the repository.

**The project is worth 30% of the total course grade**. Evaluation shifts from theoretical design to technical execution over the semester. In Milestone 1, we grade your **plan**. In Milestone 2, we grade your **understanding**. In Milestone 3, we grade your **code**.


### 3.2. Hardware vs. Simulation Requirements
To ensure a fair level of rigor between those working with physical robots and those in simulation, the following requirements apply:

* **Hardware Teams:** Must implement **one (1)** custom algorithm/module.
* **Simulation Teams:** Must implement **two (2)** custom algorithms/module.
* **The Hardware Bonus (+5 pts):** Applied to teams managing physical hardware constraints. (5% of 30 Grade Points = 1.5 Grade Points)

### 3.3. Individual Accountability
To ensure every team member contributes to the robotics engineering, technical scores are verified using the repository's metadata:

- `git shortlog -sn`: Used to track commit volume and frequency. This identifies the "velocity" of each contributor throughout the semester.

- `git blame`: Used during Milestones 2 and 3 to verify that the person claiming credit for a mathematical derivation is the same person who authored the corresponding logic in the files.


#### 3.2.1 Individual Grade Adjuster (The Audit)

At the end of **Milestone 2** and **Milestone 3**, your individual grade will be calculated as follows:

* **Multiplier 1.0:** `git shortlog` shows consistent work; `git blame` confirms authorship of claimed logic.
* **Multiplier 0.5 - 0.8:** `git shortlog` shows "Big Bang" pushes (all work done in 1 day); `git blame` shows you only edited Markdown files/comments.
* **Multiplier 0.0:** No commits found under your registered GitHub email address.


#### 3.2.2 Why use this model?

It is designed to strengthen group dynamics by clarifying roles:

- **It Prevents "The Passenger" Problem**: Sometimes, one person may perform the majority of the coding. This requirement forces team memebers to own a piece of the technical stack.

- **It Encourages "Ownership and not Isolation"**: Owning a file doesn't mean working alone! Teams should still pair-program and review each other's code. However, the person responsible for the Perception Lead role should be the one navigating the "driver's seat" when that code is written.
**
- **It Mimics Industry Standards**: In professional robotics, you are often the "Owner" of a specific repository or module. You are responsible for its bugs, its documentation, and its performance, even if your teammates contribute small fixes to it.

```note
Work performed from a shared account or a teammate's computer will not be credited to you. Always commit from your individual account to ensure your technical contribution is logged for your final grade in the Milestones 2 and 3.
```

## 4. Submission Guidelines

All project documentation must be submitted via a Markdown-based Jekyll website (hosted via GitHub Pages). The tutorial for this can be found [here](../tutorials/github_pages.md). 

This approach ensures that your technical documentation is as accessible and professional as the code residing in your git repository. By utilizing this format, you will learn how to build and maintain a static website, refine your technical writing skills, and showcase your work to a larger audience beyond the class.

---

<center>Design for failure; build for safety.</center>

---
