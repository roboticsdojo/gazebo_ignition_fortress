# Gazebo Ignition Fortress Robot - Comprehensive Guide

## Table of Contents
1. [Project Overview](#project-overview)
2. [Package Structure](#package-structure)
3. [Robot Description Files](#robot-description-files)
4. [Launch Files](#launch-files)
5. [Configuration Files](#configuration-files)
6. [World Files](#world-files)
7. [SDF (Simulation Description Format)](#sdf-simulation-description-format)
8. [Gazebo Ignition](#gazebo-ignition)
9. [SLAM (Simultaneous Localization and Mapping)](#slam-simultaneous-localization-and-mapping)
10. [ROS2 Control](#ros2-control)
11. [Navigation2 (Nav2)](#navigation2-nav2)
12. [Getting Started](#getting-started)

## Project Overview

This project implements a complete autonomous robot simulation using Gazebo Ignition, ROS2, and various robotics components. The robot is designed as a differential drive platform with LiDAR, camera sensors, and full navigation capabilities.

### Key Features
- **Differential Drive Robot**: Two-wheeled mobile platform with precise motion control
- **Multi-Sensor System**: LiDAR for SLAM and obstacle detection, camera for visual feedback
- **Full Navigation Stack**: Complete Nav2 implementation with SLAM, localization, and path planning
- **Multiple Simulation Worlds**: Various environments for testing different scenarios
- **ROS2 Control Integration**: Hardware abstraction and controller management
- **Teleoperation Support**: Multiple input methods (keyboard, joystick)

## Package Structure

```
src/ppp_bot/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── description/                # Robot URDF/XACRO files
├── launch/                     # Launch files for different scenarios
├── config/                     # Configuration files for controllers and navigation
├── worlds/                     # SDF world files for simulation environments
├── models/                     # 3D models for world objects
└── maps/                       # Pre-generated maps for navigation
```

## Robot Description Files

### `description/robot.urdf.xacro`
**Purpose**: Main robot description file that includes all robot components
- **Includes**: robot_core.xacro, lidar.xacro, camera.xacro
- **Function**: Defines the complete robot structure using XACRO macros

### `description/robot_core.xacro`
**Purpose**: Defines the basic robot chassis and drive system
- **Components**:
  - `base_link`: Root coordinate frame
  - `chassis`: Main robot body (0.3m × 0.3m × 0.15m)
  - `left_wheel_joint` & `right_wheel_joint`: Continuous rotation joints
  - `left_wheel` & `right_wheel`: Cylindrical wheels (radius: 0.05m)
- **Kinematics**: Differential drive with 0.35m wheel separation

### `description/lidar.xacro`
**Purpose**: LiDAR sensor configuration for SLAM and obstacle detection
- **Mounting**: Fixed to chassis at position (0.15, 0, 0.15)
- **Specifications**:
  - **Type**: GPU LiDAR sensor
  - **Range**: 0.08m to 20.0m
  - **Scan Rate**: 10 Hz
  - **Field of View**: 360° (640 samples)
  - **Topic**: `/scan`

### `description/camera.xacro`
**Purpose**: RGB camera for visual feedback
- **Mounting**: Fixed to chassis at position (0.305, 0, 0.08)
- **Specifications**:
  - **Resolution**: 640×480 pixels
  - **Field of View**: 1.089 radians (~62°)
  - **Update Rate**: 10 Hz

### `description/ros2_control.xacro`
**Purpose**: Hardware abstraction layer for robot control
- **System Type**: `gz_ros2_control/GazeboSimSystem`
- **Joint Interfaces**: Velocity command and state interfaces for both wheels
- **Limits**: Velocity range -10 to 10 rad/s

## Launch Files

### `launch/launch_ign.launch.py`
**Purpose**: Main launch file for Gazebo Ignition simulation
- **Components Launched**:
  - Gazebo Ignition simulator with specified world
  - Robot state publisher
  - Robot spawner
  - ROS2-Gazebo bridge
  - Controller loading (joint_state_broadcaster, diff_drive_base_controller)

### `launch/navigation.launch.py`
**Purpose**: Complete navigation stack launch
- **Components**:
  - Nav2 lifecycle manager
  - AMCL (Adaptive Monte Carlo Localization)
  - Nav2 controller, planner, and navigator
  - Nav2 costmap 2D
  - Nav2 map server

### `launch/online_async.launch.py`
**Purpose**: Online SLAM with async mapping
- **Components**: SLAM Toolbox with async mapping mode

### `launch/teleop_sim.launch.py`
**Purpose**: Teleoperation control for simulation
- **Input Methods**: Keyboard and joystick support

## Configuration Files

### `config/my_controllers.yaml`
**Purpose**: ROS2 Control configuration for robot motion
- **diff_drive_base_controller**:
  - **Wheels**: left_wheel_joint, right_wheel_joint
  - **Geometry**: 0.35m separation, 0.05m radius
  - **Limits**: ±2.0 m/s linear, ±2.0 rad/s angular
- **joint_state_broadcaster**: Publishes joint states for TF tree

### `config/nav2_params.yaml`
**Purpose**: Navigation2 stack configuration
- **AMCL Parameters**: Particle filter with 500-2000 particles
- **BT Navigator**: Behavior trees for navigation and recovery
- **Controller**: DWB (Dynamic Window Based) algorithm
- **Planner**: NavFn (Navigation Function) with A* pathfinding
- **Costmap**: Static, inflation, and obstacle layers

### `config/mapper_params_online_async.yaml`
**Purpose**: SLAM Toolbox configuration for mapping
- **Solver**: Ceres optimization with sparse normal Cholesky
- **Mapping Mode**: Online async (real-time mapping)
- **Parameters**:
  - **Resolution**: 0.05m
  - **Max Range**: 20.0m
  - **Update Interval**: 5.0s
  - **Loop Closure**: Enabled with 10 scan minimum chain

## World Files

### Available Worlds
- **`cones.sdf`**: World with traffic cones for obstacle avoidance testing
- **`maze.sdf`**: Maze environment for navigation testing
- **`room.sdf`**: Simple room environment
- **`room_with_cones.sdf`**: Room with added obstacles
- **`barrels.sdf`**: World with construction barrels
- **`moving_robot.sdf`**: Dynamic environment with moving objects
- **`empty.sdf`**: Minimal environment for basic testing

## SDF (Simulation Description Format)

### What is SDF?
SDF is an XML-based format for describing robots, sensors, and environments in simulation. It's the native format for Gazebo Ignition.

### SDF Structure
```xml
<sdf version='1.9'>
  <world name='world_name'>
    <physics>...</physics>
    <plugin>...</plugin>
    <gui>...</gui>
    <model>...</model>
  </world>
</sdf>
```

### Key SDF Elements
- **`<world>`**: Container for all simulation elements
- **`<physics>`**: Physics engine configuration
- **`<plugin>`**: System plugins (sensors, controllers, etc.)
- **`<gui>`**: User interface configuration
- **`<model>`**: Robot and object definitions
- **`<link>`**: Physical components
- **`<joint>`**: Connections between links
- **`<sensor>`**: Sensor definitions

### Launching SDF Worlds
```bash
# Launch specific world
ros2 launch ppp_bot launch_ign.launch.py world_name:=cones

# Available worlds
world_name:=cones, maze, room, room_with_cones, barrels, moving_robot, empty
```

## Gazebo Ignition

### What is Gazebo Ignition?
Gazebo Ignition is the next-generation robotics simulator, replacing the original Gazebo. It provides high-fidelity physics simulation, sensor simulation, and visualization.

### Key Features
- **Physics Engine**: Multiple physics engines (ODE, Bullet, DART)
- **Sensor Simulation**: Realistic sensor data (LiDAR, cameras, IMU)
- **Plugin System**: Extensible architecture for custom behaviors
- **GUI**: Interactive 3D visualization and control
- **ROS2 Integration**: Native ROS2 support via ros_gz packages

### Core Systems
1. **Physics System**: Handles dynamics and collision detection
2. **Sensors System**: Simulates sensor data
3. **Scene Broadcaster**: Manages 3D scene updates
4. **User Commands**: Handles user interactions
5. **Contact System**: Manages object interactions

## SLAM (Simultaneous Localization and Mapping)

### What is SLAM?
SLAM is a technique for building a map of an unknown environment while simultaneously tracking the robot's position within that map.

### SLAM Toolbox
This project uses SLAM Toolbox, a ROS2 SLAM implementation based on the Karto SLAM algorithm.

### Key Components
1. **Scan Matching**: Aligns LiDAR scans to build the map
2. **Loop Closure**: Detects when the robot revisits areas
3. **Graph Optimization**: Optimizes the map using Ceres solver
4. **Pose Graph**: Maintains robot trajectory and map consistency

### SLAM Modes
- **Mapping**: Create new maps from scratch
- **Localization**: Track position in known maps
- **Online Async**: Real-time mapping with background optimization

### Usage
```bash
# Start SLAM mapping
ros2 launch ppp_bot online_async.launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# Load existing map for localization
ros2 launch ppp_bot localization.launch.py
```

## ROS2 Control

### What is ROS2 Control?
ROS2 Control provides a standardized interface for robot hardware abstraction, allowing different controllers to work with various robot platforms.

### Architecture
1. **Hardware Interface**: Abstracts physical hardware
2. **Controller Manager**: Manages multiple controllers
3. **Controllers**: Implement specific control algorithms
4. **Resource Manager**: Handles resource allocation

### Controllers in This Project
1. **Joint State Broadcaster**:
   - Publishes joint states for TF tree
   - Required for robot visualization and navigation

2. **Differential Drive Controller**:
   - Controls left and right wheel velocities
   - Implements differential drive kinematics
   - Publishes odometry data

### Configuration
- **Update Rate**: 100 Hz
- **Wheel Separation**: 0.35m
- **Wheel Radius**: 0.05m
- **Velocity Limits**: ±2.0 m/s linear, ±2.0 rad/s angular
- **Acceleration Limits**: ±1.0 m/s² linear, ±1.0 rad/s² angular

## Navigation2 (Nav2)

### What is Nav2?
Navigation2 is the ROS2 navigation stack that provides autonomous navigation capabilities including path planning, obstacle avoidance, and localization.

### Core Components
1. **AMCL (Adaptive Monte Carlo Localization)**:
   - Particle filter localization
   - Tracks robot position in known maps
   - Adapts to sensor noise and environment changes

2. **Nav2 Controller**:
   - Follows planned paths
   - Implements obstacle avoidance
   - Uses DWB (Dynamic Window Based) algorithm

3. **Nav2 Planner**:
   - Generates optimal paths to goals
   - Uses NavFn (Navigation Function) algorithm
   - Considers costmap information

4. **Nav2 Behavior Tree Navigator**:
   - Orchestrates navigation behaviors
   - Handles recovery actions
   - Manages navigation state machine

5. **Nav2 Costmap 2D**:
   - Maintains occupancy grid
   - Includes static, dynamic, and inflation layers
   - Provides cost information for planning

### Navigation Stack
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   AMCL          │    │   Nav2 Planner  │    │   Nav2 BT       │
│   (Localization)│    │   (Path Planning)│    │   (Navigation)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   TF Tree       │    │   Costmap 2D    │    │   Nav2 Controller│
│   (Transforms)  │    │   (Environment) │    │   (Path Following)│
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Key Features
- **Multi-robot Support**: Can handle multiple robots
- **Recovery Behaviors**: Spin, back up, clear costmap
- **Dynamic Reconfiguration**: Runtime parameter changes
- **Behavior Trees**: Flexible navigation logic
- **Costmap Layers**: Static, dynamic, inflation, obstacle

## Getting Started

### Prerequisites
```bash
# Install ROS2 Humble
sudo apt install ros-humble-desktop

# Install Gazebo Ignition
sudo apt install ignition-fortress

# Install ROS2-Gazebo bridge
sudo apt install ros-humble-ros-gz

# Install Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install SLAM Toolbox
sudo apt install ros-humble-slam-toolbox
```

### Building the Project
```bash
# Clone and build
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
colcon build --packages-select ppp_bot
source install/setup.bash
```

### Basic Usage
```bash
# Launch simulation with navigation
ros2 launch ppp_bot launch_ign.launch.py world_name:=cones

# In another terminal, launch navigation
ros2 launch ppp_bot navigation.launch.py

# Start SLAM mapping
ros2 launch ppp_bot online_async.launch.py

# Teleoperate the robot
ros2 launch ppp_bot teleop_sim.launch.py
```

### Usage Examples

#### 1. Basic Simulation
```bash
# Launch empty world
ros2 launch ppp_bot launch_ign.launch.py world_name:=empty

# Check robot state
ros2 topic echo /joint_states
ros2 topic echo /scan
```

#### 2. SLAM Mapping
```bash
# Launch world and start mapping
ros2 launch ppp_bot launch_ign.launch.py world_name:=cones
ros2 launch ppp_bot online_async.launch.py

# Drive robot around to create map
ros2 launch ppp_bot teleop_sim.launch.py

# Save map when complete
ros2 run nav2_map_server map_saver_cli -f ~/cones_map
```

#### 3. Navigation with Known Map
```bash
# Launch with existing map
ros2 launch ppp_bot launch_ign.launch.py world_name:=cones
ros2 launch ppp_bot navigation.launch.py

# Set initial pose in RViz2
# Send navigation goals
```

### Debug Commands
```bash
# Check available topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /cmd_vel

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor node graph
ros2 node list
ros2 node info /node_name
```

## Troubleshooting

### Common Issues
1. **Gazebo Ignition not found**: Install ignition-fortress
2. **ROS2-Gazebo bridge errors**: Check topic remappings
3. **Controller loading failures**: Verify ros2_control configuration
4. **Navigation not working**: Check TF tree and costmap
5. **SLAM not mapping**: Verify LiDAR data and parameters

### Performance Optimization
1. **Reduce LiDAR samples**: Lower scan resolution
2. **Increase update intervals**: Reduce computational load
3. **Use simpler physics**: Choose appropriate physics engine
4. **Optimize costmap size**: Balance accuracy and performance

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

*This comprehensive guide covers all aspects of the Gazebo Ignition Fortress Robot project. For additional support, refer to the ROS2, Gazebo Ignition, and Navigation2 documentation.*
