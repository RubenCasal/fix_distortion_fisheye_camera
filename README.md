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

![Fisheye Distortion Correction](fisheye_correction.gif)

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
