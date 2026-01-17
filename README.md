# Rosie Robot - Motion Planning & Perception System

A complete ROS2 motion planning and perception system for the Rosie mobile manipulator robot. Integrates YOLOv8 object detection with MoveIt2 motion planning for autonomous pick-and-place operations.

<div align="center">
  <a href="https://www.youtube.com/watch?v=qHLM9LW5f4Y">
    <img src="demo.gif" alt="Demo" />
  </a>
  
  <br> <p>
    <a href="https://www.youtube.com/watch?v=qHLM9LW5f4Y">
      <b>🎥 Click here to watch the full video on YouTube</b>
    </a>
  </p>
</div>
## Overview

This workspace contains:
- **YOLOv8 Object Detection**: Real-time detection of objects from camera feed
- **MoveIt2 Motion Planning**: Trajectory planning for robotic arm
- **ROS2 Control**: Hardware interface for arm and gripper
- **RViz2 Visualization**: Real-time robot and planning visualization

## System Architecture

```
Isaac Sim / Camera Input
    ↓
YOLOv8 Detection (/Yolov8_Inference)
    ↓
Arm Control (/target_point) → MoveIt2 Planning
    ↓
ROS2 Control → Hardware Execution
    ↓
RViz2 Visualization
```

## Quick Start

### Prerequisites
- ROS2 Humble (Ubuntu 22.04 LTS recommended)
- Docker (optional, for containerized setup)
- Python 3.10+

### Installation

1. **Clone the repository:**
```bash
git clone <repo-url> Rosie-Robot
cd Rosie-Robot
```

2. **Install dependencies:**
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

3. **Build the workspace:**
```bash
colcon build --symlink-install
```

4. **Source the setup:**
```bash
source install/setup.bash
```

### Running the System

**Terminal 1 - Start YOLOv8 object detection:**
```bash
ros2 launch yolov8_obb yolov8_obb.launch.py
```

**Terminal 2 - Launch robot controller:**
```bash
ros2 launch rosie_moveit_config rosie_controller.launch.py
```

**Terminal 3 - Launch motion planning + arm control:**
```bash
ros2 launch rosie_moveit_config rosie_moveit_launch.launch.py
```

The system will:
- Display RViz2 visualization
- Listen to `/target_point` topic for pick-and-place commands
- Execute motion plans via MoveIt2

### Docker Setup (Optional)

**Build Docker image:**
```bash
cd /path/to/Rosie-Robot
docker build -f docker/Dockerfile -t rosie_image .
```

**Run Docker container:**
```bash
./start_rosie.sh
```

Inside the container:
```bash
cd /workspace/rosie
colcon build --symlink-install
source install/setup.bash
ros2 launch yolov8_obb yolov8_obb
```

## Package Structure

```
Rosie-Robot/ (GitHub Repository Root)
├── src/                           # ROS2 packages
│   ├── yolov8_obb/                    # YOLOv8 detection node
│   │   ├── scripts/
│   │   │   └── yolov8_obb_publisher.py    # Detection publisher
│   │   ├── launch/
│   │   │   └── yolov8_obb.launch.py        # Launch file
│   │   └── package.xml
│   │
│   ├── yolov8_obb_msgs/               # Custom detection messages
│   │   ├── msg/
│   │   │   ├── InferenceResult.msg        # Single detection
│   │   │   └── Yolov8Inference.msg        # Collection of detections
│   │   └── package.xml
│   │
│   ├── rosie_description/             # Robot URDF model
│   │   ├── urdf/
│   │   │   ├── model.urdf
│   │   │   └── meshes/                    # 3D mesh files (STL)
│   │   └── package.xml
│   │
│   └── rosie_moveit_config/           # Motion planning configuration
│       ├── launch/
│       │   ├── rosie_controller.launch.py         # Hardware control
│       │   └── rosie_moveit_launch.py             # Main launch file
│       ├── scripts/
│       │   └── arm_control_from_UI.py             # Pick-and-place controller
│       ├── config/
│       │   ├── rosie_V1.urdf.xacro                # Robot model (parametric)
│       │   ├── rosie_V1.srdf                      # Semantic description
│       │   ├── kinematics.yaml                    # Inverse kinematics
│       │   ├── moveit_controllers.yaml            # MoveIt configuration
│       │   ├── ros2_controllers.yaml              # ROS2 control config
│       │   └── motion_planning.rviz               # RViz layout
│       └── package.xml
│
├── docker/                        # Docker setup
│   ├── Dockerfile
│   ├── ros_entrypoint.sh
│   └── slam_params/               # SLAM configuration
│
├── README.md                      # This file
├── .gitignore                     # Git ignore patterns
├── .dockerignore                  # Docker build ignore patterns
└── start_rosie.sh                 # Startup script (relative paths)
```

## Core Launch Files

### 1. YOLOv8 Detection
**File:** `yolov8_obb/launch/yolov8_obb.launch.py`

Launches object detection node that:
- Subscribes to `/image_raw` (camera feed)
- Runs YOLOv8 inference
- Publishes `/Yolov8_Inference` (detections)
- Publishes `/inference_result` (annotated image)

### 2. Robot Controller
**File:** `rosie_moveit_config/launch/rosie_controller.launch.py`

Starts hardware control:
- `robot_state_publisher` - URDF to TF transforms
- `controller_manager` - Hardware interface
- Arm controller spawner
- Gripper controller spawner

### 3. Motion Planning + Arm Control
**File:** `rosie_moveit_config/launch/rosie_moveit_launch.py`

Launches complete system:
- Includes `rosie_controller.launch.py`
- Starts `arm_control_from_UI.py` (main controller)
- Launches RViz2 visualization

## Arm Control Script

**File:** `rosie_moveit_config/scripts/arm_control_from_UI.py`

Main control logic that:
- Subscribes to `/target_point` (x, y, z coordinates)
- Uses MoveIt2 to plan trajectories
- Executes autonomous pick-and-place sequence:
  1. Move above object
  2. Open gripper
  3. Move down to object
  4. Close gripper
  5. Move up with object
  6. Move to drop location
  7. Open gripper (release)

## ROS Topics

### Input Topics
- `/target_point` (Float64MultiArray) - Pick-and-place target [x, y, z]
- `/image_raw` (Image) - Camera feed from Isaac Sim

### Output Topics
- `/Yolov8_Inference` (Yolov8Inference) - Detected objects
- `/inference_result` (Image) - Annotated camera image
- `gripper_bool` (Bool) - Gripper state (open/close)

## Configuration Files

### kinematics.yaml
Inverse kinematics solver configuration

### moveit_controllers.yaml
MoveIt2 motion planning parameters

### ros2_controllers.yaml
ROS2 control interface configuration

### motion_planning.rviz
Pre-configured RViz layout with robot visualization



## Dependencies

Key ROS2 packages:
- `moveit` - Motion planning framework
- `ros2_control` - Hardware interface
- `robot_state_publisher` - TF broadcasting
- `rviz2` - Visualization
- `ultralytics` - YOLOv8 library

Install with:
```bash
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
```

## Docker Setup

Optional containerized execution:
```bash
./start_rosie.sh
```

## Development

### Building
```bash
colcon build --symlink-install
```

### Testing
```bash
colcon test
```

### Cleaning
```bash
rm -rf build install log
```

## Resources

### Assets
- **[Robot Meshes (STL files)](https://drive.google.com/file/d/1WAdeyv-nnFQlLWe24muNCMv4s2-Gqwvz/view?usp=drive_link)** - 3D models for Rosie robot (~70MB)
- **[Isaac Sim USD File](https://drive.google.com/file/d/1bep9C88b1P6gQsWAgvrjjt-YRDIcd1CU/view?usp=drive_link)** - Complete simulation environment

### Learning Resources
- **[Pick and Place Simulation Using MoveIt and Yolov8 OBB](https://www.youtube.com/watch?v=ypr3RtJzgKI)** - Reference tutorial (UI and OBB implementation based on this)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [ros2_control](https://control.ros.org/)
