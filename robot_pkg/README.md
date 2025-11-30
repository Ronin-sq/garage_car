# Robot Package (robot_pkg)

This package contains the robot model and simulation setup for a differential drive robot.

## Overview

The robot_pkg provides:
- URDF/SDF models of a differential drive robot
- Controller configuration for the robot
- Launch files for simulation in Gazebo and visualization in RViz
- Ready-to-use configurations for ROS 2 control systems

## Package Structure

```
robot_pkg/
├── config/                 # Configuration files
│   ├── controller.yaml     # ROS 2 controller configurations
│   ├── ros2_controller.yaml# Advanced ROS 2 controller configurations
│   ├── robot.rviz          # RViz configuration
│   └── rviz_config.rviz    # Alternative RViz configuration
├── launch/                 # Launch files
│   ├── gazebo_sim.launch.py    # Launch simulation in Gazebo
│   ├── preview.launch.py       # Preview robot with controllers
│   └── robot.launch.py         # Launch robot visualization
├── urdf/                   # URDF robot models
│   ├── robot.urdf          # Simple URDF model
│   ├── robot.xacro         # Basic xacro model
│   ├── robot_car.xacro     # Detailed robot model
│   ├── robot_car_new.xacro # Updated robot model
│   └── robot/              # Modular robot components
└── world/                  # Gazebo world files
```

## Launch Files

### gazebo_sim.launch.py
Launches the robot in a Gazebo simulation environment:
- Spawns the robot in the simulated world
- Starts robot_state_publisher node
- Launches Gazebo with the simulation world
- Runs RViz for visualization

Usage:
```bash
ros2 launch robot_pkg gazebo_sim.launch.py
```

### preview.launch.py
Launches the robot with controller configuration for testing:
- Starts robot_state_publisher node
- Initializes controller manager with diff drive controller
- Spawns joint state broadcaster
- Loads diff_drive_controller
- Runs RViz for visualization

Usage:
```bash
ros2 launch robot_pkg preview.launch.py
```

### robot.launch.py
Launches basic robot visualization without simulation:
- Starts robot_state_publisher node
- Starts joint_state_publisher node
- Runs RViz for visualization

Usage:
```bash
ros2 launch robot_pkg robot.launch.py
```

## Controllers

The package includes configuration for three main controllers:

1. **diff_drive_controller**: Controls the differential drive system
   - Configured with left and right wheel joints
   - Wheel separation: 0.3m (controller.yaml) or 0.1m (ros2_controller.yaml)
   - Wheel radius: 0.04m (controller.yaml) or 0.032m (ros2_controller.yaml)
   - Publishes odometry transformations

2. **joint_state_broadcaster**: Broadcasts joint states
   - Provides joint position and velocity information

3. **effort_controllers**: Controls joint efforts
   - Allows direct effort control of wheel joints
   - Useful for custom control algorithms

## Configuration

The package contains two controller configuration files:

### [controller.yaml](config/controller.yaml)
Basic configuration for ROS 2 controllers:
- Update rate: 100Hz
- Uses simulation time
- Differential drive controller settings
- Simple odometry parameters

### [ros2_controller.yaml](config/ros2_controller.yaml)
Advanced configuration for ROS 2 controllers:
- Update rate: 100Hz
- Uses simulation time
- Enhanced differential drive controller settings with more parameters
- Effort controller configuration for direct joint control
- Detailed covariance settings for odometry
- Additional parameters like wheel separation multiplier and open loop control

## Dependencies

This package depends on the following ROS 2 packages:
- `gazebo_ros_pkgs` - for Gazebo simulation
- `robot_state_publisher` - for publishing robot states
- `joint_state_publisher` - for joint state publishing
- `controller_manager` - for managing controllers
- `diff_drive_controller` - for differential drive control
- `joint_state_broadcaster` - for broadcasting joint states
- `effort_controllers` - for effort-based joint control
- `rviz2` - for visualization

## Usage

To use this package:

1. Launch the simulation:
   ```bash
   ros2 launch robot_pkg gazebo_sim.launch.py
   ```

2. Or preview with controllers:
   ```bash
   ros2 launch robot_pkg preview.launch.py
   ```

3. Control the robot using the diff_drive_controller topics:
   - Publish velocity commands to `/cmd_vel`