---
layout: default
title: Perception
parent: Assignments
sort: 1
---

# Assignment 1: Semantic Landmark Extraction and Classification

In this assignment, you will process point cloud data recorded in rosbags to implement a perception pipeline that detects and locates cylinders within the robot’s environment.

## Prerequisites
* **HSV Color Space:** Read the tutorial on the [HSV Color Space](../tutorials/hsv_color_space.md).
* **Data Playback:** Review the [ROS 2 Bag Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) (Recording and Playing Back Data).

### ROS2 Simulation Environment Setup
You will not be using a physical robot for this assignment.

Instruct your VM to use the start its own Discovery Server during your simulation/emulation:
```bash
set-ros-env sim
```

```warning
Make sure you close all open terminals and start fresh after this step to make sure the new settings are sourced!
```

```note
**Bare Metal Configuration:** If you are using a bare metal Ubuntu instead of a VM and this step does not work, please start a Canvas discussion and describe your issue.
```

## 1. Learning Objectives
* Learn **3D point cloud preprocessing** (Box Filtering and Voxel Downsampling).
* Implement **RANSAC** from first principles for plane segmentation and geometric fitting.
* Implement **Normal Estimation** using Singular Value Decomposition (SVD) with NumPy.
* Map physical objects to semantic roles (Home, Obstacle, Target) using **HSV color logic**.
* Understand ROS 2 data structures for 3D visualization (`MarkerArray` and `PointCloud2`).

## 2. Technical Constraints
* **Allowed Libraries:** `NumPy`, `math`, and standard ROS 2 packages (`sensor_msgs`, `geometry_msgs`, `tf2`, `visualization_msgs`).
* **OpenCV Usage:** Restricted **strictly** to color space conversion (`cvtColor`) and basic masking if required. 
* **Banned Libraries:** `Open3D`, `PCL`, `scikit-learn`, `SciPy` spatial/clustering modules (use only the provided `cKDTree` for neighbor search).
* **The "Pure NumPy" Rule:** All geometric logic, including RANSAC loops, distance calculations, and Linear Solvers, must be written in NumPy. 
* **Academic Integrity:** If you use a high-level library to bypass the algorithmic work, or ask a LLM to program your assignment, you will fail the oral checkoff.

## 3. ROS Bags

Download the rosbags from [here](https://arizonastateu-my.sharepoint.com/:f:/g/personal/vsangar2_asurite_asu_edu/IgBHgvuC9WBMR65AStY6-LxdAeQ-QOa9wD4Se8DddV8UWSU?e=Fmi8KP). Unzip the bags for use. Begin with the first bag; its simplicity makes it an ideal starting point.

| name | description | size | md5sum |
| --- | --- | --- | --- |
| `rgbd_bag_0` | Single green cylinder | 106M | `b176478b3d7525ef789b37195aa4af20` |
| `rgbd_bag_1` | Robot moving around different cylinders | 334M | `9c6a8ae75f74c691b6cdacf45f8dc8d6` |
| `rgbd_bag_2` | Three cylinders | 145M | `2bdc3d8db0925123916cf9a54869d0a6` |

To ensure your Python node can keep up with the data processing, play the bag in a loop at a reduced rate:

```bash
ros2 bag play <your_bag_file>.mcap --loop --rate 0.5
```

## 4. Visualization (RViz2)
RViz2 (ROS Visualization) is a 3D visualizer that allows you to see what the robot "sees." It takes abstract data—sensor readings, camera feeds, and coordinate frames—and renders them into a 3D environment.

#### 4.1. **Launch and Global Configuration**

1. **Run RViz:** Type `rviz2` in your VM terminal.
2. **Global Options:** Set the **Fixed Frame** to `oakd_rgb_camera_optical_frame`.

```warning
The Fixed Frame must match the `frame_id` in your PointCloud2 messages (`oakd_rgb_camera_optical_frame`). If the Fixed Frame is set incorrectly, your 3D data will not render or will appear with a "Status: Error" in the displays panel.
```

Add displays to visualize pointclouds, markers and images.

#### 4.2 **Saving and Loading Configurations**

Manually adding topics every time you launch RViz is inefficient. You should save your setup to a `.rviz` configuration file.

* **To Save:** Go to **File > Save Config As** to save the configuration file.
* **To Load:** Next time you open RViz, go to **File > Open Config**, or launch it directly from the terminal with the configuration flag:

```bash
rviz2 -d path/to/your/config.rviz
```

RViz is a great tool for debugging. Experiment with the GUI: change the background color, particle sizes, etc.

## 4. Skeleton Code
The following script is a good starting point that handles all the "plumbing": loading your bag files into memory and providing an interactive 3D interface. You are encouraged to modify, edit, or delete any part of this logic; it is designed to be flexible, and there are many valid programming/mathematical approaches to successfully implement this assginment. Make sure to use NumPy operations to avoid slow Python `for` loops.


```python
import argparse
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# ==========================================
# CONFIGURATION CLASS
# ==========================================
class PipelineConfig:
    """
    Holds parameters for the point cloud processing pipeline.
    You can add, change or remove any of the parameters here.
    """
    def __init__(self):
        # Topic settings
        self.topic = '/oakd/points' # Topic containing the pointcloud
        
        # Voxel Downsampling
        self.voxel_size = 0.02
        
        # Passthrough/Box Filter (Min/Max XYZ)
        self.box_min = np.array([-1.0, -0.6, 0.2]) 
        self.box_max = np.array([ 1.0,  0.6, 2.0]) 

        # Plane RANSAC
        self.floor_dist = 0.02
        self.target_normal = np.array([0, 1, 0]) # Assuming Y-up for floor
        self.normal_thresh = 0.85
        
        # Cylinder RANSAC
        self.cyl_radius = 0.055
        self.max_cylinders = 3  

# ==========================================
# VISUALIZER CLASS
# ==========================================
class CylinderVisualizer:
    """
    Handles the creation and publishing of RViz MarkerArrays to represent 
    detected cylinders.
    """
    def __init__(self, publisher):
        self.pub_markers = publisher

    def create_cylinder_marker(self, center, radius, rgb, marker_id, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        
        m.pose.position.x = float(center[0])
        m.pose.position.y = float(0.0) # Snap to floor level for visualization
        m.pose.position.z = float(center[2])
        
        # Rotate cylinder to stand upright
        m.pose.orientation.x = 0.7071
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 0.7071
        
        m.scale.x = float(radius * 2.0)
        m.scale.y = float(radius * 2.0)
        m.scale.z = 0.4 
        
        m.color.r = float(rgb[0])
        m.color.g = float(rgb[1])
        m.color.b = float(rgb[2])
        m.color.a = 0.8
        return m

    def publish_viz(self, cylinders, frame_id):
        ma = MarkerArray()
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        ma.markers.append(clear_marker)

        for i, (model, rgb, name) in enumerate(cylinders):
            center, _, radius = model
            marker = self.create_cylinder_marker(center, radius, rgb, 2000 + i, frame_id)
            ma.markers.append(marker)
        
        self.pub_markers.publish(ma)

# ==========================================
# PIPELINE LOGIC 
# ==========================================
class CylinderPipeline:
    def __init__(self, cfg: PipelineConfig):
        self.cfg = cfg
    
    def rgb_to_hsv(self, r, g, b):
        """
        Converts a single RGB point to HSV color space.
        
        :param r: Red component (0.0 - 1.0)
        :param g: Green component (0.0 - 1.0)
        :param b: Blue component (0.0 - 1.0)
        :return: Tuple (h, s, v) where H is [0, 360], S and V are [0, 1]
        """
        mx = max(r, g, b)
        mn = min(r, g, b)
        df = mx - mn
        
        # Calculate Hue
        if mx == mn:
            h = 0
        elif mx == r:
            h = (60 * ((g - b) / df) + 360) % 360
        elif mx == g:
            h = (60 * ((b - r) / df) + 120) % 360
        elif mx == b:
            h = (60 * ((r - g) / df) + 240) % 360
            
        # Calculate Saturation
        s = 0 if mx == 0 else (df / mx)
        
        # Calculate Value
        v = mx
        
        return h, s, v

    def get_neighbors(self, pts, queries, k=15):
        """
        Calculates k-nearest neighbors using a KDTree.
        
        :param pts: The source point cloud (Nx3).
        :param queries: The points for which we want neighbors (Mx3).
        :param k: Number of neighbors to find.
        :return: Indices of neighbors in the 'pts' array.
        """
        if len(pts) < k: return None
        tree = cKDTree(pts)
        _, idxs = tree.query(queries, k=k)
        return idxs

    def box_filter(self, pts, colors):
        """
        Removes points outside the specified XYZ bounding box.
        
        :param pts: Input XYZ array.
        :param colors: Input RGB array.
        :return: Tuple of (filtered_pts, filtered_colors).
        """
        pass

    def downsample(self, pts, colors):
        """
        Reduces point cloud density using a voxel grid approach.
        
        Implementation Hint: Convert points to integer coordinates by dividing 
        by voxel_size, then use np.unique to find one point per voxel.
        """
        pass

    def estimate_normals(self, pts, k=15):
        """
        Estimates a surface normal for every point.
        
        Implementation Hint: 
        1. For each point, find k-neighbors.
        2. Compute the Singular Value Decomposition (SVD), using np.linalg.svd, of the centered neighbors.
        3. The normal is the eigenvector corresponding to the smallest eigenvalue.
        """
        pass

    def find_plane_ransac(self, pts, iters=100):
        """
        Fits a plane model (ax + by + cz + d = 0) to the cloud using RANSAC.
        
        Implementation Hint: 
        1. Sample 3 random points to define a plane.
        2. Calculate the normal and check if it aligns with self.cfg.target_normal.
        3. Count how many points are within self.cfg.floor_dist of the plane.
        4. Return the model with the most inliers.
        """
        pass

    def find_single_cylinder(self, pts, normals, iters=300):
        """
        Fits a cylinder model to the remaining points using RANSAC.
        
        Implementation Hint:
        1. Sample 2 points and their normals.
        2. The cylinder axis is roughly the cross product of the two normals.
        3. Check axis alignment with the vertical.
        4. Project points and find distance to the axis; compare to self.cfg.cyl_radius.
        """
        pass

# ==========================================
# ROS NODE
# ==========================================
class CylinderProcessorNode(Node):
    def __init__(self):
        super().__init__('cylinder_processor_node')
        self.cfg = PipelineConfig()
        self.pipeline = CylinderPipeline(self.cfg)
        
        # Publishers for debugging the pipeline stages in RViz
        self.pub_stage0 = self.create_publisher(PointCloud2, 'pipeline/stage0_box', 10)
        self.pub_stage3 = self.create_publisher(PointCloud2, 'pipeline/stage3_candidates', 10)
        
        # Marker publisher for the final detection results
        marker_pub = self.create_publisher(MarkerArray, 'viz/detections', 10)
        self.visualizer = CylinderVisualizer(marker_pub)
        
        self.sub = self.create_subscription(PointCloud2, self.cfg.topic, self.listener_callback, 10)
        
    def numpy_to_pc2_rgb(self, pts, colors, frame_id):
        """
        Converts Nx3 XYZ coordinates and Nx3 RGB color arrays into a ROS 2 PointCloud2 message.
        
        This utility handles the conversion of floating-point spatial data and the packing
        of three 8-bit color channels (R, G, B) into a single 32-bit float field, which is 
        the standard format for RGB point clouds in ROS and RViz.

        :param pts: A numpy array of shape (N, 3) containing [x, y, z] coordinates.
        :param colors: A numpy array of shape (N, 3) containing [r, g, b] values (0.0 to 1.0).
        :param frame_id: The TF frame string (e.g., 'camera_link') for the message header.
        :return: A sensor_msgs/PointCloud2 message ready for publishing.
        """
        msg = PointCloud2()
        msg.header.frame_id, msg.height, msg.width = frame_id, 1, len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 16, True
        msg.row_step = 16 * len(pts)
        c = (np.clip(colors, 0, 1) * 255).astype(np.uint32)
        rgb_packed = (255 << 24) | (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
        data = np.hstack([pts.astype(np.float32), rgb_packed.view(np.float32).reshape(-1, 1)])
        msg.data = data.tobytes()
        return msg
    
    def listener_callback(self, msg):
        """
        Main ROS Callback. Orchestrates the flow from PointCloud2 to Cylinder detection.
        """
        frame_id = msg.header.frame_id
        stride = msg.point_step // 4 
        raw_data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, stride)
        
        # 1. Extraction: Get XYZ points and Filter NaNs
        pts = raw_data[:, :3]
        finite_mask = np.all(np.isfinite(pts), axis=1)
        pts = pts[finite_mask]
        
        # 2. Color Extraction: Decode packed float32 RGB values
        rgb_uint32 = raw_data[finite_mask, 4].view(np.uint32)
        raw_colors = np.vstack([
            ((rgb_uint32 >> 16) & 0xFF) / 255.0, # Red
            ((rgb_uint32 >> 8) & 0xFF) / 255.0,  # Green
            (rgb_uint32 & 0xFF) / 255.0          # Blue
        ]).T

        # TODO: Implement the call sequence:
        # pts_box, colors_box = self.pipeline.box_filter(pts, raw_colors)
        # pts_v, colors_v = self.pipeline.downsample(pts_box, colors_box)
        # ...
        
        # Final detections format: list of ((center, axis, radius), rgb_color, name)
        detected_cylinders = [] 
        
        self.visualizer.publish_viz(detected_cylinders, frame_id)

def main():
    rclpy.init()
    node = CylinderProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## 5. The Pipeline

### Task 0: Preprocessing (Performance Optimization)
Raw OAK-D clouds are too dense for pure-Python processing. Also, you will notice that the OAK-D Lite has a lot of noise as the points are furhter away from the camera. It is advised that you perform one or both of the filtering steps:
1. **Box Filter:** Implement a "Pass-through" filter to discard points outside the robot's immediate workspace (e.g., $$x > 3m$$ or $$\|y\| > 1.5m$$).

2. **Downsampling:** Use a Voxel Grid approach. Divide space into 2cm cubes and average the points within each cube using `np.unique`.

### Task 1: Floor and Ceiling Removal (RANSAC)
The lab contains floor, ceiling, and table surfaces that must be removed to isolate the cylinders.
1. **Algorithm:** Implement RANSAC to find the dominant horizontal plane.
2. **Constraint:** The plane normal $$\vec{n} = [a, b, c]$$ must be approximately vertical ($$[0, 1, 0]$$). (Refer [OAK-D Lite Camera Frame](https://imgur.com/520twhc)) Reject planes that represent walls.


### Task 2: Euclidean Clustering
Group the remaining points into distinct objects.
1. **Logic:** Points within a distance threshold $$d$$ (e.g., 0.1m) belong to the same cluster.
2. **Implementation:** Use vectorized NumPy operations (like broadcasting) to calculate distances. Avoid nested Python loops where possible to maintain real-time performance.

### Task 3: Cylinder Detection (Cylinder RANSAC)

1. **Logic**: Randomly sample two points and their estimated normals. The cylinder axis is the cross product of these two normals.

2. **Inlier Counting**: Project all points onto the axis. Points whose perpendicular distance to the axis is within a certain threshold are considered inliers.

3. **Iterative Detection**: Once a cylinder is found, remove its inliers and repeat the process to find the next one.

```note
Use `np.cross` and `np.linalg.norm` to calculate distances for the entire cloud at once rather than looping through points individually.
```

### Task 4: Semantic Labeling via Point Color**
For each verified cylinder cluster, calculate the average color of the points, use the `rgb_to_hsv()` to convert the color to HSV space, and use a range on the values to group into Red, Green and Blue. Visualize their filtered point cloud in RViz with the right color. 

```note
If the color data in the original pointcloud appears misaligned with the physical cylinder in RViz, it may be due to hardware sync issues or motion blur. Use the color data from the points that physically form the cylinder shape.
```

## 6. Submission and Checkoff

**Git Submission:** Provide a Git repository link, a specific commit hash, and a `.zip` of that commit on Canvas.


### 6.1. Grading Rubic

| Category | Criteria | Points |
| --- | --- | --- |
| **Data Preprocessing** | Successful implementation of **Box Filter** and **Voxel Downsampling**. The node must handle the raw cloud without significant lag at 0.5x playback speed. | **3** |
| **Plane Segmentation** | Robust **RANSAC** implementation that identifies and removes the floor/ceiling. | **3** |
| **Cylinder Detection** | Correct implementation of **Cylinder RANSAC** using point-normal pairs. Must successfully calculate the axis and identify inliers based on the target radius. | **4** |
| **Semantic Labeling** | Accurate conversion from **RGB to HSV**. Correctly identifies the semantic role (Home, Obstacle, Target) based on hue thresholds. | **3** |
| **ROS Integration** | Proper publishing of `MarkerArray` for RViz. Transformation/Frame IDs are correct, and markers are persistent and stable. | **2** |

### 6.2. Deductions

* **-5 Points:** Use of "Banned Libraries" (Open3D, PCL, or SciPy's RANSAC/Clustering functions). When in doubt, check with the instructor!
* **-X Points:** You must be able to explain your code (data structure, code flow, algorithm implementation, etc). The oral checkoffs will happen after your submission, based on the code you submitted. 
