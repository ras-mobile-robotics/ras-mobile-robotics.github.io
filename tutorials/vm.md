---
layout: default
title: "Virtual Machine"
parent: Tutorials
sort: 2
---

## Enable 3D Hardware Acceleration

In a robotics context, "Hardware Acceleration" typically refers to offloading computation from the CPU to the GPU. It is essential for:

* **Gazebo / Isaac Sim (Simulation):** These physics engines calculate light, shadows, and physics collisions. Software rendering makes real-time simulation impossible.
* **RViz2 (Visualization):** RViz2 uses **OpenGL** to render complex sensor data (point clouds, robot models, laser scans). Without acceleration, the CPU must emulate these graphics ("Software Rendering"), leading to massive lag.

---

### Setup Guide

1. Make sure you shutdown (cleanly) the VM.
2. Change the **VM Settings:**
    - **Display:** Check **Accelerate 3D graphics**.
    - **Graphics Memory:** Set to at least **2GB** (recommended for ROS 2).

    > **Note:** If your hardware has limited VRAM (less than 2GB), set this value to the maximum VRAM allowed by your slider.

3. Boot into the VM, run the following command to install Open VM Tools:

    ```bash
    sudo apt update && sudo apt install open-vm-tools-desktop
    ```

3. **Reboot** the VM.

`open-vm-tools-desktop` is a package that optimizes the user experience when running a Linux VM (the "Guest") inside a hypervisor like VMware (the "Host"). It enables the "quality of life" features you see in the GUI:
- Shared Clipboard: Enables Copy & Paste between your host computer and the VM.
- Drag and Drop: Allows you to drag files directly into the VM window.
- Improved Graphics: Includes drivers that help the VM communicate with your physical GPU for the 3D acceleration mentioned earlier in your guide.

---

### Verification & Troubleshooting 

#### 1. Verify 3D Hardware Acceleration is Active

Run this command in the terminal:

```bash
glxinfo | grep "direct rendering"
```

* **Success:** It should say `Yes`.
* **Failure:** If it says `No` or the command is missing, run `sudo apt install mesa-utils` first.

Check the renderer being used:

```bash
glxinfo | grep "renderer string"

```

* **Good:** `SVGA3D` (VMware) or `VMware7` (VirtualBox).
* **Bad:** `llvmpipe` (This means it's using the CPU for graphics, which causes lag).

#### 2. The "Black Screen" or "Flickering" Fix (Ubuntu 24.04 Specific)

If you see a black screen or weird visual artifacts:

1. On the Login screen, click the **Gear Icon** in the bottom right.
2. Select **Ubuntu on Xorg**.
3. Log in. This uses the older X11 window system, which is much more stable for VM 3D drivers.

---

#### 3. Gazebo Simulation Performance

To check the performance, look for the **Real Time Factor (RTF)** at the bottom right corner of the Gazebo window. If you expand the toolbar, you will see `Sim Time`, `Real Time`, and `Iterations`.

**Understanding RTF:**

* **RTF = 1.00:** Perfect performance. 1 second in the simulation equals 1 second in the real world.
* **RTF < 1.00:** The simulation is lagging ("slow motion").
* *Example:* An RTF of `0.50` means the simulation is running at half speed (1 sim-second takes 2 real-seconds). This is common in VMs.


* **RTF > 1.00:** The simulation is running faster than real-time (fast forward).

**Goal:** In a VM, an RTF between **0.7 and 1.0** is acceptable. If it drops below **0.4**, your robot's navigation stack may time out (see below).

---

#### 4. Controller Timeout Errors

If your RTF is low, you might see repeated warning logs like this in your terminal:

```bash
[gazebo-1] [WARN] [diffdrive_controller]: Ignoring the received message (timestamp 38.08) because it is older than the current time by 0.50 seconds...
```

**What this means:**
The robot's safety controller expects commands (`cmd_vel`) to arrive instantly. Because your VM is lagging, the commands are arriving "late" (older than the 0.5s safety limit), so the robot ignores them to prevent dangerous behavior.

**The Fix:**
You need to increase the timeout tolerance.

1. Open the configuration file (you will need `sudo`):
```bash
sudo nano /opt/ros/jazzy/share/irobot_create_control/config/control.yaml
```
2. Find the parameter `cmd_vel_timeout` (default is usually `0.5`).
3. Change it to `2.5` (or higher if the warnings persist).
4. Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`), then restart the simulation.

---

#### 5. Other Common Issues

* **Laptop Power Settings:** Make sure your laptop is plugged into power and your OS power mode is set to **"High Performance"**. Windows 11 often throttles the GPU on battery, which effectively "breaks" the 3D bridge to the VM.

---


## Software Rendering for Gazebo
If you have already attempted the steps in [3D Hardware Acceleration](#enable-3d-hardware-acceleration) and Gazebo still flickers or crashes, the best solution is to use **Software Rendering**.

By forcing the system to use software-based OpenGL, you bypass the buggy virtualized 3D acceleration drivers of the VM. But now all the rendering is handled by the CPU, so the simulation performance will get affected.

### Option 1: Basic Software Rendering

Run this in your terminal before launching the TurtleBot 4 simulation:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Option 2: Software Rendering with Version Override

If the simulation still fails to load or the sensors (Lidar/Camera) cause a crash, try overriding the OpenGL version:

```bash
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1
```

---

## Expand VM Hard Disk

```tip
ROS2 logs might take a large amount of disk space over time.
Clean the workspace log folder by running: `colcon clean log`. If that does not free up space, proceed with the turotial to expand your VM disk.
```

Expanding your disk on an Ubuntu 24.04 VM involves two main stages: increasing the "hardware" limit in VMware and then using the built-in GUI to claim that space for `/dev/sda2` (the disk that the Ubuntu VM uses).

```warning
Backup your code/data in the VM before you proceed.
```

### Phase 1: Increase Disk Size in VMware Player

You must perform these steps while the Virtual Machine is completely powered off.

1. **Shut down** your Ubuntu VM.
2. In the VMware Player library, select your VM and click **Edit virtual machine settings**.
3. Select **Hard Disk (NVMe)** or **Hard Disk (SCSI)** from the device list.
4. On the right-hand side, look for the "Capacity" section and click the **Expand...** button.
5. Enter the **new maximum disk size** (e.g., if you want 100GB total, enter 100).
6. Click **Expand**. Once you see the "Disk expansion successful" message, click **OK** and power on your VM.

---

### Phase 2: Resize `/dev/sda2` using GParted

Once Ubuntu boots up, it will see the extra space as "Unallocated," meaning it isn't assigned to any partition yet.

1. **Open GParted:** Search for "GParted" in your Ubuntu applications menu and launch it. (You will be prompted for your password).
2. **Locate the Partition:** Find **`/dev/sda2`** in the list. You should see a gray box labeled **Unallocated** immediately to the right of it.
3. **Resize the Partition:**
* Right-click on the **`/dev/sda2`** row.
* Select **Resize/Move**.
4. **Extend to Fill:** In the window that pops up, use your mouse to grab the **right edge** of the partition box and drag it all the way to the right until there is no gray space left.
5. **Confirm Resize:** Click the **Resize/Move** button at the bottom of that window.
6. **Apply Changes:** At this point, the change is only "pending." Click the **Checkmark icon** (✔) in the top toolbar to apply the operations.
7. **Wait for Completion:** Once the process finishes, click **Close**.

---

### Phase 3: Verification
To ensure your ROS 2 environment and system tools now have access to the full space, you can quickly check the disk status:

* Open the **"Disk Usage Analyzer"** (standard on Ubuntu) to see the new capacity.
* Alternatively, run `df -h /` in the terminal to see the updated size for your root directory.

