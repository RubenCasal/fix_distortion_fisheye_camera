# Fix Distortion Fisheye Camera - ROS2 Package

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![C++](https://img.shields.io/badge/C%2B%2B-14-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)

## 📌 Overview
This **ROS2 package** processes **fisheye images from the Intel RealSense T265** to correct distortion and project images into rectilinear space. It provides:
- **C++ Node (`t265_node`)**: Publishes **odometry, IMU, and fisheye images**.
- **Python Nodes (`defisheye_fix_distortion`, `defisheye_rectiliniar_projection`)**:  
  - `defisheye_fix_distortion.py`: Applies **fisheye distortion correction**.  
  - `defisheye_rectiliniar_projection.py`: Projects the fisheye image into **rectilinear space**.  
- **Launch Files**: Automatically start the necessary nodes.

![Fisheye Distortion Correction](defisheye.gif)

---

## 📸 Features
✔ **Real-time Odometry & IMU Data**  
✔ **Fisheye Image Publishing**  
✔ **Fisheye to Rectilinear Projection**  
✔ **Distortion Removal using OpenCV**  
✔ **Seamless ROS2 Integration (Humble)**  

---

## 🚀 Installation & Setup

### **1️⃣ Clone the Repository**
Ensure you have the **Librealsense2** library installed.

```bash
cd ~/ros2_ws/src
git clone git@github.com:RubenCasal/fix_distortion_fisheye_camera.git
cd ~/ros2_ws
colcon build --packages-select fix_distortion_fisheye_camera
source install/setup.bash
```

## 🏁 Running the Package
Run Nodes Separately
Start the T265 Node (C++)
```
ros2 run fix_distortion_fisheye_camera t265_node
```
Start the Fisheye Distortion Correction Node (Python)
```
ros2 run fix_distortion_fisheye_camera defisheye_fix_distortion.py
```
Start the Rectilinear Projection Node (Python)
```
ros2 run fix_distortion_fisheye_camera defisheye_rectiliniar_projection.py
```
Run with Launch Files
Launch Fisheye Distortion Correction:
```
ros2 launch fix_distortion_fisheye_camera defisheyed_launch.py
```
Launch Rectilinear Projection:
```
ros2 launch fix_distortion_fisheye_camera rectiliniar_projection.py
```

## 📡 Published Topics

This package publishes multiple topics for **navigation and image processing**.

| **Topic Name**                                  | **Message Type**                 | **Description**                                 |
|------------------------------------------------|----------------------------------|-------------------------------------------------|
| `/rs_t265/odom`                                | `nav_msgs/msg/Odometry`         | Odometry data (position & pose).               |
| `/rs_t265/imu`                                 | `sensor_msgs/msg/Imu`           | IMU data (gyro & acceleration).                |
| `/rs_t265/fisheye_left`                        | `sensor_msgs/msg/Image`         | Left fisheye image from the T265.              |
| `/rs_t265/fisheye_right`                       | `sensor_msgs/msg/Image`         | Right fisheye image from the T265.             |
| `/rs_t265/undistorted`                         | `sensor_msgs/msg/Image`         | Fisheye image after distortion correction.     |
| `/rs_t265/defisheye_rectiliniar_projection_node` | `sensor_msgs/msg/Image`         | Rectilinear projection of the fisheye image.  |
| `/tf`                                          | `tf2_msgs/msg/TFMessage`        | TF transformations for localization.           |

## 🎥 Visualizing Results in RViz2
### 1️⃣ Launch RViz2

Start RViz2:
```
rviz2
```
### 2️⃣ Add Required Displays

In RViz2:

    Click "Add" → "By Topic"
    Select:
        /rs_t265/odom → Odometry (trajectory visualization)
        /rs_t265/imu → IMU (orientation data)
        /rs_t265/fisheye_left → Image (raw camera feed)
        /rs_t265/undistorted → Image (fisheye distortion correction result)
        /rs_t265/defisheye_rectiliniar_projection_node → Image (rectilinear projection result)
        /tf → TF (transform visualization)


## 🛠 How Fisheye Distortion Correction Works

### 🔹 What is Fisheye Distortion?
Fisheye lenses provide an **ultra-wide field of view**, often exceeding **180°**, making them useful for robotics, drones, and SLAM applications. However, they introduce **radial distortion**, causing straight lines to appear **curved**, especially toward the image edges.  

This distortion can negatively impact:
- **Computer Vision Algorithms** (e.g., object detection, feature matching).  
- **Localization & Mapping** (e.g., SLAM, 3D reconstruction).  
- **Navigation** (e.g., obstacle avoidance in autonomous systems).  

To ensure **accurate scene perception**, we must **correct fisheye distortion** before processing images.

### 🔹 How Fisheye Distortion Correction Works
Fisheye distortion correction involves remapping pixels to **remove radial distortion**, effectively "straightening" curved edges.  
This transformation is achieved using **intrinsic camera parameters**, which model the lens distortion.  

#### 🔹 Steps in the Correction Process
- **Camera Calibration**  
  - The camera is **calibrated** using a known pattern (e.g., a chessboard).  
  - Intrinsic parameters, such as **focal length, principal point, and distortion coefficients**, are estimated.  
  - These parameters are then used to compute a **correction transformation matrix**.  

- **Remapping Pixels**  
  - Each pixel is mapped to its **corrected position** based on the transformation matrix.  
  - OpenCV's `cv2.undistort()` function applies this remapping, reducing curvature.  
  - Lookup tables ensure fast and efficient correction.  

- **Generating an Undistorted Image**  
  - The corrected image is reconstructed using **`cv2.remap()`**.  
  - Straight lines remain straight, making the image **more suitable for further processing** in robotics, SLAM, and computer vision tasks.  

---

## 🛠 How Rectilinear Projection Works

### 🔹 Why Rectilinear Projection?
Fisheye distortion correction only **removes radial distortion** but does not change the **perspective of the image**.  
In many cases, we need to go further and **convert the fisheye view into a normal-looking image**—a process known as **rectilinear projection**.  

This is important because:
- Fisheye images **compress distant objects**, making measurements inaccurate.  
- Many **computer vision algorithms** assume images come from **pinhole cameras** (not fisheye).  
- SLAM and mapping require **undistorted, perspective-correct images** for feature tracking.  

### 🔹 How Rectilinear Projection Works
Rectilinear projection **transforms the curved fisheye image into a standard pinhole camera view** by **mapping spherical coordinates to Cartesian coordinates**.

#### 🔹 Steps in the Projection Process
- **Convert Fisheye Image to Polar Coordinates**  
  - The fisheye image represents a **spherical perspective**.  
  - Each pixel is converted to **polar coordinates** (`r, θ`) instead of traditional `(x, y)`.  

- **Reproject to Cartesian Space**  
  - The **spherical projection is "unwrapped" into a flat, rectilinear image**.  
  - This ensures objects appear **proportionally correct**, eliminating fisheye warping.  

- **Interpolation & Final Projection**  
  - Since pixel positions are now **non-uniform**, interpolation (bilinear or bicubic) is applied for a smooth transformation.  
  - The final result is a **normal-looking image**, similar to what a regular **pinhole camera** would capture.  

This transformation is critical for **SLAM, 3D reconstruction, and visual perception in robotics**, where **accurate spatial representation** is essential.  

  - Uses **bilinear interpolation** to preserve quality.  
  - Produces a **normal-looking image** similar to a **pinhole camera**.  
