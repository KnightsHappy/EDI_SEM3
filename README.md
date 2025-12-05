# EDI_SEM3 - ROS2 Autonomous Navigation System

A complete ROS2 Humble workspace for autonomous mobile robot navigation in a warehouse environment using Nav2, AMCL localization, and Gazebo simulation.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Known Issues](#known-issues)

## 🎯 Overview

This project implements a complete autonomous navigation stack for a differential drive robot in a warehouse simulation. The robot can:
- Spawn automatically in Gazebo with proper initial pose
- Localize itself using AMCL (Adaptive Monte Carlo Localization)
- Navigate autonomously to goal positions while avoiding obstacles
- Plan optimal paths using the Nav2 navigation stack

## ✨ Features

- **Automatic Robot Spawning**: Robot spawns at predefined position with correct orientation
- **Automatic Localization**: AMCL initializes automatically without manual pose estimation
- **Optimized Nav2 Stack**: Configured for minimal odometry drift and accurate localization
- **GUI Navigation Tool**: Custom GUI for sending navigation goals (workaround for RViz tool issues)
- **Complete Simulation**: Warehouse environment with obstacles and navigation challenges

## 📦 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble Hawksbill
- **Simulator**: Gazebo Ignition (Fortress)

### Required ROS2 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros-gz \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3* \
  python3-colcon-common-extensions \
  python3-rosdep
```

### Python Dependencies
```bash
pip3 install tkinter  # For navigation GUI
```

## 🚀 Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/KnightsHappy/EDI_SEM3.git
   cd EDI_SEM3
   ```

2. **Install dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**
   ```bash
   colcon build
   source install/setup.bash
   ```

## 🎮 Usage

### Launch the Complete Navigation System

```bash
source install/setup.bash
ros2 launch eyantra_warehouse task2_nav.launch.py
```

**What happens:**
- **T=0s**: Gazebo launches with warehouse world
- **T=2s**: Robot spawns at position (-1.5339, -6.6156, yaw=1.57)
- **T=8s**: Nav2 stack initializes
- **T=9s**: Initial pose automatically published, AMCL localizes
- **T=10s**: RViz opens with robot visible and localized
- **Ready**: Navigation system ready for goal commands

### Send Navigation Goals

#### Option 1: GUI Tool (Recommended)
```bash
# In a new terminal
source install/setup.bash
ros2 run eyantra_warehouse nav_goal_gui.py
```
- Enter X, Y coordinates and yaw angle (degrees)
- Click "Send Goal"
- Use preset buttons for quick goals

#### Option 2: Command Line
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: -0.5, y: -6.6, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

#### Option 3: Test Script
```bash
ros2 run eyantra_warehouse test_nav_goal.py
```

### Verify System Status

**Check Nav2 lifecycle nodes:**
```bash
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger
# Should return: success=True
```

**Monitor navigation topics:**
```bash
# Check if path is being planned
ros2 topic echo /plan --once

# Check velocity commands
ros2 topic echo /cmd_vel

# Check localization
ros2 topic echo /particle_cloud
```

## 📁 Project Structure

```
src/
├── ebot_description/          # Robot URDF/XACRO description
│   ├── urdf/                  # Robot model files
│   └── launch/                # Robot spawning launch files
│
├── eyantra_warehouse/         # Main navigation package
│   ├── config/
│   │   ├── nav2_params.yaml   # Nav2 configuration (AMCL, planners, controllers)
│   │   ├── nav2.rviz          # RViz visualization config
│   │   └── bridge.yaml        # Gazebo-ROS bridge config
│   ├── launch/
│   │   ├── task2_nav.launch.py  # Main launch file (complete system)
│   │   └── nav2.launch.py       # Nav2 stack only
│   ├── maps/
│   │   ├── campus_map.pgm     # Occupancy grid map
│   │   └── campus_map.yaml    # Map metadata
│   ├── scripts/
│   │   ├── publish_initial_pose.py  # Auto initial pose publisher
│   │   ├── nav_goal_gui.py          # GUI navigation tool
│   │   └── test_nav_goal.py         # Test navigation script
│   └── worlds/                # Gazebo world files
│
├── ur_description/            # UR robot descriptions
└── ur_simulation_gz/          # UR simulation support
```

## ⚙️ Configuration

### Key Configuration Files

#### 1. Nav2 Parameters (`config/nav2_params.yaml`)

**AMCL (Localization):**
- Reduced motion model noise (`alpha1-4: 0.05`) for better drift correction
- Increased particles (`max: 5000, min: 1000`) for accurate localization
- Frequent updates (`update_min_a: 0.1, update_min_d: 0.1`)

**Controller:**
- DWB local planner with obstacle avoidance
- Max velocity: 0.5 m/s linear, 1.0 rad/s angular
- Goal tolerance: 0.25m position, 0.25 rad orientation

**Planner:**
- NavFn global planner
- Tolerance: 0.5m
- Unknown space allowed

#### 2. Launch Configuration (`launch/task2_nav.launch.py`)

**Timing sequence:**
- Robot state publisher: 0s
- Robot spawn: 2s
- Nav2 bringup: 8s (allows Gazebo clock sync)
- Initial pose publisher: 9s
- RViz: 10s

#### 3. Initial Pose (`scripts/publish_initial_pose.py`)

Automatically publishes initial pose matching spawn position:
- Position: (-1.5339, -6.6156, 0.055)
- Orientation: yaw = 1.57 rad (90°)

## 🔧 Troubleshooting

### Robot Not Visible in RViz
- **Check**: Robot description topic is `/ebot/robot_description`
- **Fix**: Ensure robot_state_publisher is running with correct namespace

### No Particle Cloud
- **Check**: AMCL is active and initial pose was published
- **Fix**: Manually set initial pose or restart launch file

### Navigation Goals Don't Work
1. **Verify Nav2 is active:**
   ```bash
   ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger
   ```
2. **Use GUI tool** instead of RViz Nav2 Goal button
3. **Check topics:**
   ```bash
   ros2 topic list | grep -E "(goal|plan)"
   ```

### Odometry Drift
- AMCL parameters are optimized to minimize drift
- Ensure laser scan data is good (`ros2 topic echo /scan`)
- Robot needs visible features (walls, obstacles) to localize

### Build Errors
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --packages-select eyantra_warehouse
source install/setup.bash
```

## ⚠️ Known Issues

### RViz Nav2 Goal Tool
The RViz "Nav2 Goal" button may not publish goals due to plugin compatibility issues.

**Workaround:** Use the provided GUI tool:
```bash
ros2 run eyantra_warehouse nav_goal_gui.py
```

### BT Navigator Plugins
Some behavior tree plugins from newer Nav2 versions are not available in Humble. The configuration has been updated to exclude:
- `nav2_are_error_codes_present_condition_bt_node`
- `nav2_would_a_smoother_help_condition_bt_node`
- `nav2_assisted_teleop_action_bt_node`
- `nav2_path_expiring_timer_condition`

## 📝 Important Notes

1. **Always source the workspace** before running commands:
   ```bash
   source install/setup.bash
   ```

2. **Wait for initialization**: Allow ~10 seconds after launch for all nodes to initialize

3. **Coordinate system**: All positions are in the `map` frame

4. **Goal coordinates**: Hover over the map in RViz to see coordinates for goal selection

## 🤝 Contributing

This is an academic project for EDI Semester 3. For issues or improvements, please create an issue or pull request.

## 📄 License

This project is for educational purposes.

## 🙏 Acknowledgments

- ROS2 Navigation2 team
- Gazebo simulation team
- e-Yantra robotics competition

---

**Repository**: https://github.com/KnightsHappy/EDI_SEM3

**Last Updated**: December 2025
