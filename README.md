# UR Robot Hand-Eye Calibration

This project provides a hand-eye calibration pipeline for UR robotic arms. It computes a fixed transformation matrix from the robot's end-effector to the camera.

## Overview

A fixed AprilTag is placed on the ground as a reference target. The robot is manually controlled to 20 different poses where the tag is visible in the camera frame. At each pose, the tag's coordinate data is recorded. After all samples are collected, the program computes the end-effector-to-camera transformation matrix.

## Getting Started

A convenience script is provided to launch all required programs at once:

```bash
bash launch.sh
```

## Calibration Procedure

1. Place the AprilTag flat on the ground in a fixed, stable position. It must remain stationary throughout the entire process.
2. Run `launch.sh` to start all required programs.
3. Manually control the UR robot to move the arm to a pose where the tag is visible in the camera frame.
4. Take a sample — the system records the tag's coordinate data at the current pose.
5. Repeat steps 3–4 for **20 different poses**, varying position and orientation each time.
6. Once all 20 samples are collected, the program automatically computes the end-effector-to-camera transformation matrix.

## Known Issues

**Controller input lag**: When controlling the robot via the controller, commands may occasionally experience delays, causing the control panel to become temporarily unresponsive. If this happens, wait a moment before retrying.

## References

Tsai, Roger Y., and Reimar K. Lenz. *"A new technique for fully autonomous and efficient 3D robotics hand/eye calibration."* Robotics and Automation, IEEE Transactions on 5.3 (1989): 345-358.
