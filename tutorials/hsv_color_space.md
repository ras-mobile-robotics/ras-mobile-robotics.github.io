---
layout: default
title: HSV Color Space
parent: Tutorials
sort: 11
---

A color space is a mathematical model that describes how colors can be represented as numbers. In robotics, choosing the right color space is the difference between a robot that functions only in a pitch-black room and one that works in the real world.

### 1. The RGB Color Space (The "Electronic" Frame)

RGB is the standard for hardware (cameras, monitors). It is an **additive** model where Red, Green, and Blue light are mixed. It is a 3D cube where the axes are $$R$$, $$G$$ and $$B$$.

One of the bigegst drawback of this representation is that it is **computationally fragile** for perception. Because the channels are highly correlated, changing the brightness of a light source changes the values of all three components.
Hence in Compute Vision, RGB is only used for raw data ingestion, and almost never for thresholding or classification.

---

### 2. The HSV Color Space (The "Perceptual" Frame)

HSV was designed to align with how humans perceive color. It transforms the RGB cube into a cylinder or cone.

* **Hue ($$H$$):** The "Angle" of color. This is the most critical channel for your assignment.
* **Saturation ($$S$$):** The "Purity." Low saturation looks like gray/white; high saturation is a deep, vivid color.
* **Value ($$V$$):** The "Intensity" or brightness.

It allows for **Lighting Invariance**. By setting a wide threshold for $$V$$ but a narrow threshold for , your robot can identify a "Blue" cylinder whether it is in a shadow or under a bright fluorescent lamp.

When your robot looks at a **Red** cylinder, the RGB values will fluctuate wildly as the robot moves and the camera exposure adjusts. However, the **Hue** will remain centered around $$0\degree$$ (or $$180\degree$$  in OpenCV).

| Space | Best For... | Worst For... |
| --- | --- | --- |
| **RGB** | Displaying images, Raw data storage | Outdoor lighting, Shadows, Object detection |
| **HSV** | Color-based segmentation, Object tracking | High-fidelity photo editing |

---

While the OpenCV `cv2.cvtColor` function is the most convenient, "better" usually depends on whether you mean **mathematically more robust** or **faster for large datasets**.

In the context of your students' Point Cloud processing, a "better" approach is a **vectorized NumPy implementation**. This allows them to convert an entire cluster of thousands of points to HSV in a single operation without the overhead of calling OpenCV functions inside a Python `for` loop.

### 1. The Vectorized NumPy Implementation

If your students have a NumPy array of shape  representing the BGR colors of their point cloud, they can use this vectorized logic. It is much faster and helps them understand the "Pure NumPy" requirement of the course.

```python
import numpy as np

def rgb_to_hsv(rgb_array):
    """
    Vectorized RGB to HSV conversion for (N, 3) arrays.
    Returns standard mathematical units.
    Input:  rgb_array [0, 255]
    Output: H [0, 360], S [0, 1], V [0, 1]
    """
    # Normalize RGB to [0, 1]
    rgb = rgb_array.astype(float) / 255.0
    r, g, b = rgb[:, 0], rgb[:, 1], rgb[:, 2]

    # Compute Value and Delta
    v = np.max(rgb, axis=1)
    m = np.min(rgb, axis=1)
    delta = v - m

    # Calculate Saturation (0.0 to 1.0)
    s = np.zeros_like(v)
    s[v != 0] = delta[v != 0] / v[v != 0]

    # Calculate Hue (0.0 to 360.0)
    h = np.zeros_like(v)
    has_color = (delta != 0)
    
    idx_r = (v == r) & has_color
    idx_g = (v == g) & has_color
    idx_b = (v == b) & has_color

    h[idx_r] = 60 * (((g[idx_r] - b[idx_r]) / delta[idx_r]) % 6)
    h[idx_g] = 60 * (((b[idx_g] - r[idx_g]) / delta[idx_g]) + 2)
    h[idx_b] = 60 * (((r[idx_b] - g[idx_b]) / delta[idx_b]) + 4)

    return np.stack([h, s, v], axis=1)

```

### 4. The Red Problem
The "Red Problem" occurs because **Hue is a circular** property, not a linear one. In a $$0 \degree$$ – $$360\degree$$ system, Red is located at the "seam" where the scale loops.

On a standard $$360\degree$$ color wheel:

- $$0 \degree$$ is pure Red.
- $$120\degree$$  is pure Green.
- $$240\degree$$  is pure Blue.
- $$360\degree$$  is ... pure Red again.

### 4.1. The Threshold Issue

A single range fails to capture Red. You must use a split condition to catch both "Orange-Red" and "Purple-Red":

* **Incorrect:** `if h < 20` (Misses values like )
* **Correct:** `is_red = (h < 20) | (h > 340)`

### 2. The Mean Disaster

If your cluster has points at $$2\degree$$ and $$258\degree$$, a standard average results in $$180\degree$$ (Cyan).

* **The Math:** $$\frac{22+358}{2}=180$$.
* **The Reality:** Your robot sees a Red cylinder but labels it "Green/Cyan" because the average jumped across the circle.

The Result: 180∘ is Cyan/Green. Your robot will look at a red cylinder and be convinced it is seeing the opposite color.

### 3. The Vector Solution

To find the true average, treat Hues as vectors on a unit circle:

1. **Convert** Hue $$\theta$$ to $$(x,y)$$ using $$cos(\theta)$$ and $$sin(\theta)$$.
2. **Average** the $$x$$ and $$y$$ values.
3. **Convert back** to an angle using `arctan2`.

```python
def circular_mean(h_array):
    rad = np.degree2rad(h_array)
    avg_x, avg_y = np.mean(np.cos(rad)), np.mean(np.sin(rad))
    return np.rad2degree(np.arctan2(avg_y, avg_x)) % 360

```