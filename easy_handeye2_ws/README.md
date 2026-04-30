# easy_handeye2_ws

ROS2 workspace for the hand-eye calibration pipeline. See the [main README](../README.md) for project overview and calibration procedure.

## Workspace Structure

```
easy_handeye2_ws/
├── src/
│   ├── easy_handeye2/            # Upstream calibration library (submodule)
│   └── aruco_detector/           # Custom ArUco detection ROS2 package
│       ├── aruco_detector/
│       │   └── detect_vis_node.py
│       └── launch/
│           └── aruco_detect.launch.py
```

## Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **Intel RealSense SDK** + `realsense2_camera` ROS2 package
- **OpenCV** with ArUco module (`python3-opencv`)
- **UR Robot Driver** publishing TF frames

## Build

```bash
cd easy_handeye2_ws
rosdep install -iyr --from-paths src
colcon build
source install/setup.bash
```

## ArUco Detector Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_id` | `27` | ArUco marker ID to detect |
| `marker_length_m` | `0.19` | Physical marker side length (meters) |
| `pose_method` | `aruco_pnp` | `aruco_pnp` (recommended) or `depth_center` |
| `publish_coordinate_convention` | `optical` | `optical` or `camera_link` |
| `publish_tf_frame` | `true` | Broadcast marker as TF frame |
| `marker_frame_name` | `aruco_tag` | TF frame name for the marker |
| `show_window` | `true` | Show OpenCV debug window |
| `publish_debug_image` | `true` | Publish debug image to `/aruco_tracker/debug_image` |

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/aruco_tracker/pose` | `geometry_msgs/PoseStamped` | Detected marker pose |
| `/aruco_tracker/debug_image` | `sensor_msgs/Image` | Annotated debug visualization |

## Acknowledgements

- **[easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)** by Marco Esposito (LGPL v3)
- **[OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)**



