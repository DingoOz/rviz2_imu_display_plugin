# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an F-16 style Head-Up Display (HUD) RViz2 plugin that visualizes IMU data with authentic aircraft symbology. The plugin creates a realistic F-16 HUD interface showing pitch ladder, aircraft symbol, and flight attitude information in real-time from sensor_msgs/Imu messages.

## Build and Development Commands

### Building the Package
```bash
# Build the package (from workspace root)
colcon build --packages-select f16_hud_rviz_plugin

# Source the workspace after building
source install/setup.bash
```

### Testing Commands
```bash
# Launch complete demo with RViz2 and test IMU publisher
ros2 launch f16_hud_rviz_plugin hud_demo.launch.py

# Manual testing - start test IMU publisher
ros2 run f16_hud_rviz_plugin imu_test_publisher

# Manual testing - launch RViz2 with pre-configured HUD
rviz2 -d config/hud_config.rviz

# Check if plugin loads correctly
ros2 pkg list | grep f16_hud
```

### Linting and Code Quality
```bash
# Run linting tests (if BUILD_TESTING=ON)
colcon test --packages-select f16_hud_rviz_plugin
colcon test-result --verbose
```

## Architecture Overview

### Core Components

1. **F16HudDisplay** (`f16_hud_display.cpp/.hpp`): Main RViz2 plugin class
   - Inherits from `rviz_common::Display`
   - Manages ROS2 subscription to IMU topics
   - Handles plugin properties (scale, transparency, color, topic)
   - Coordinates between IMU processing and visual rendering

2. **F16HudVisual** (`f16_hud_visual.cpp/.hpp`): Ogre3D rendering engine
   - Creates and manages Ogre3D scene nodes and geometry
   - Renders authentic F-16 HUD elements (pitch ladder, aircraft symbol, side tapes)
   - Handles real-time visual updates based on attitude data

3. **ImuProcessor** (`imu_processor.cpp/.hpp`): IMU data processing
   - Converts quaternions to Euler angles (roll, pitch, yaw)
   - Handles coordinate frame transformations
   - Processes sensor_msgs/Imu messages

4. **imu_test_publisher** (`imu_test_publisher.cpp`): Test utility
   - Generates realistic F-16 flight patterns for testing
   - Publishes sensor_msgs/Imu messages with predefined maneuvers

### Plugin Architecture
- Uses Qt5 for property interfaces and RViz2 integration
- Registered via `plugins_description.xml` as "f16_hud_rviz_plugin/F16HudDisplay"
- Follows standard RViz2 plugin patterns with onInitialize/onEnable/onDisable lifecycle

### Dependencies
- **Core ROS2**: rclcpp, sensor_msgs, geometry_msgs, tf2
- **RViz2**: rviz_common, rviz_rendering, rviz_default_plugins
- **Qt**: Qt5 Core and Widgets for UI components
- **Build**: ament_cmake, pluginlib

## File Structure Patterns

### Source Organization
```
src/
├── f16_hud_display.cpp      # Main plugin class, ROS2 integration
├── f16_hud_visual.cpp       # Ogre3D rendering, HUD graphics
├── imu_processor.cpp        # IMU data processing utilities
└── imu_test_publisher.cpp   # Standalone test data generator
```

### Header Organization  
```
include/f16_hud_rviz_plugin/
├── f16_hud_display.hpp      # Plugin interface, properties
├── f16_hud_visual.hpp       # Rendering interface
└── imu_processor.hpp        # Data processing interface
```

### Configuration Files
- `config/hud_config.rviz`: Pre-configured RViz2 setup for testing
- `launch/hud_demo.launch.py`: Complete demo launch configuration
- `plugins_description.xml`: RViz2 plugin registration

## Development Guidelines

### Code Standards
- C++20 standard with modern practices
- Qt5 integration for RViz2 compatibility
- Thread-safe IMU data handling with mutex protection
- Real-time rendering considerations for smooth HUD updates

### Plugin Properties
- Topic: IMU data source (default: `/imu/data`)
- Scale: HUD size multiplier (0.1-5.0, default: 1.0)
- Alpha: Transparency (0.0-1.0, default: 0.8)
- Color: HUD color scheme (default: green)

### Testing Approach
The plugin includes comprehensive testing via `imu_test_publisher` which generates:
- Level flight with oscillations (0-10s)
- Banking turns ±30° roll (10-20s) 
- Climb maneuvers (20-30s)
- Barrel roll sequences (30-40s)

### IMU Data Processing
- Subscribes to `sensor_msgs/msg/Imu`
- Processes quaternion orientation to Euler angles
- Updates HUD visualization at RViz2 frame rate
- Handles coordinate frame transformations for aircraft display conventions