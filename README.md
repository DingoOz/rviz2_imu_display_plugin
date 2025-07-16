# F-16 HUD RViz2 Plugin

An authentic F-16 style Head-Up Display (HUD) plugin for RViz2 that visualizes IMU data with realistic aircraft symbology.

## Features

- **Authentic F-16 HUD Elements:**
  - Pitch ladder with degree markings (-30° to +30°)
  - Fixed aircraft symbol (crosshair with wings)
  - Side tapes for airspeed and altitude display
  - Heading display
  - Green-on-dark color scheme

- **Real-time IMU Integration:**
  - Subscribes to `sensor_msgs/Imu` messages
  - Converts quaternions to Euler angles
  - Real-time attitude visualization

- **Configurable Properties:**
  - Scale factor
  - Transparency/Alpha
  - Color customization
  - IMU topic selection

## Testing the Plugin

### Method 1: Build and Test (Recommended)

1. **Build the package:**
   ```bash
   cd /path/to/your/ros2_workspace
   colcon build --packages-select f16_hud_rviz_plugin
   source install/setup.bash
   ```

2. **Launch the demo:**
   ```bash
   ros2 launch f16_hud_rviz_plugin hud_demo.launch.py
   ```

This will start:
- RViz2 with the HUD plugin pre-configured
- A test IMU publisher generating realistic F-16 flight patterns

### Method 2: Manual Testing

1. **Start the test IMU publisher:**
   ```bash
   ros2 run f16_hud_rviz_plugin imu_test_publisher
   ```

2. **Launch RViz2:**
   ```bash
   rviz2 -d config/hud_config.rviz
   ```

3. **Or add the plugin manually:**
   - Open RViz2
   - Click "Add" in the Displays panel
   - Select "f16_hud_rviz_plugin/F16HudDisplay"
   - Configure the topic to `/imu/data`

### Method 3: Use Real IMU Data

If you have real IMU hardware:

1. **Start your IMU driver:**
   ```bash
   ros2 run your_imu_driver imu_node
   ```

2. **Launch RViz2 and add the HUD plugin:**
   ```bash
   rviz2
   ```
   - Add the F16HudDisplay
   - Set the topic to your IMU topic (e.g., `/imu/data`)

### Method 4: Command Line Testing

**Publish test IMU data manually:**
```bash
ros2 topic pub /imu/data sensor_msgs/msg/Imu '{
  header: {frame_id: "base_link"},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}'
```

**Check if the plugin loads:**
```bash
ros2 pkg list | grep f16_hud
ros2 interface show sensor_msgs/msg/Imu
```

## Expected Test Results

When testing, you should see:

1. **HUD Elements:**
   - Green pitch ladder lines
   - Central aircraft symbol
   - Side information panels
   - Bottom heading display

2. **Dynamic Behavior:**
   - Pitch ladder moves vertically with pitch changes
   - Entire display rotates with roll changes
   - Heading updates with yaw changes

3. **Test Pattern Sequence (with test publisher):**
   - 0-10s: Level flight with gentle oscillations
   - 10-20s: Banking turns (±30° roll)
   - 20-30s: Climb maneuvers (pitch up)
   - 30-40s: Barrel roll sequence
   - 40s+: Pattern repeats

## Troubleshooting

**Plugin doesn't appear in RViz2:**
- Verify the package is built: `colcon build --packages-select f16_hud_rviz_plugin`
- Source the workspace: `source install/setup.bash`
- Check plugin registration: `ros2 pkg xml f16_hud_rviz_plugin`

**No IMU data:**
- Check topic: `ros2 topic echo /imu/data`
- Verify publisher: `ros2 topic info /imu/data`
- Check QoS settings if using different IMU drivers

**HUD not visible:**
- Ensure the plugin is enabled in the Displays panel
- Check transparency/alpha settings
- Verify the fixed frame is set correctly
- Try adjusting the scale property

**Performance issues:**
- Reduce the IMU publishing rate
- Lower the RViz2 frame rate
- Adjust scale to reduce visual complexity

## Plugin Properties

| Property | Description | Default | Range |
|----------|-------------|---------|-------|
| Topic | IMU data topic | `/imu/data` | Any valid topic |
| Scale | Size multiplier | 1.0 | 0.1 - 5.0 |
| Alpha | Transparency | 0.8 | 0.0 - 1.0 |
| Color | HUD color | Green (0,255,0) | RGB values |

## Dependencies

- ROS2 Humble or newer
- RViz2
- Qt5
- sensor_msgs
- geometry_msgs
- tf2

## File Structure

```
f16_hud_rviz_plugin/
├── src/
│   ├── f16_hud_display.cpp     # Main plugin class
│   ├── f16_hud_visual.cpp      # Ogre3D rendering
│   ├── imu_processor.cpp       # IMU data processing
│   └── imu_test_publisher.cpp  # Test data generator
├── include/f16_hud_rviz_plugin/
│   ├── f16_hud_display.hpp
│   ├── f16_hud_visual.hpp
│   └── imu_processor.hpp
├── launch/
│   └── hud_demo.launch.py      # Demo launch file
├── config/
│   └── hud_config.rviz         # RViz2 configuration
├── CMakeLists.txt
├── package.xml
└── plugins_description.xml
```