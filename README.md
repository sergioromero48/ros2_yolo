# YOLO ROS2 Package

This package implements YOLO object detection using ROS2. It subscribes to a camera feed, processes the images using YOLO, and outputs the results with detected objects highlighted.

## Installation

1. Clone the repository into your ROS2 workspace:
   ```bash
   git clone git@github.com:sergioromero48/ros2_yolo.git ~/ros2_ws/src/yolo_pkg
   ```

2. Install any required dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --symlink-install --packages-select yolo_pkg
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Running the Package

To run the YOLO object detection with the camera stream:

1. **Launch the camera and YOLO nodes together:**
   ```bash
   ros2 launch yolo_pkg yolo_camera_launch.py
   ```

This will launch both the camera feed node and the YOLO node for object detection.

2. **If you need to run the nodes individually:**

   - Start the camera node:
     ```bash
     ros2 run camera_ros camera_node
     ```

   - Start the YOLO node:
     ```bash
     ros2 run yolo_pkg yolo_test
     ```

## Launch File

The `yolo_camera_launch.py` launch file initializes both the camera node and the YOLO node. It is configured to handle the camera stream and process it with YOLO for object detection.

To use the launch file:
```bash
ros2 launch yolo_pkg yolo_camera_launch.py
```

This will set up both the camera and YOLO nodes in a single launch process.
