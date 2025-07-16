# F-16 HUD Display RViz2 Plugin Plan

## Project Overview
Create a ROS2 package that implements an F-16 style Primary Flight Display (PFD) as an RViz2 plugin. The plugin will consume IMU data and display authentic F-16 HUD symbology within RViz2, allowing integration with other robotics visualisation tools. The display will show aircraft orientation in real-time with green graphics on dark background, mimicking actual F-16 HUD symbology.

## Requirements
- **Target Platform:** Ubuntu 24.04 LTS
- **ROS2 Distribution:** Humble or newer
- **Graphics Framework:** Qt5/Qt6 (RViz2 native)
- **Language:** Modern C++20
- **Input:** IMU data via `sensor_msgs/Imu` messages
- **Display:** RViz2 plugin panel, green on dark background

## Dependencies
```bash
# ROS2 dependencies
sudo apt install ros-humble-rviz2 ros-humble-rviz-common ros-humble-rviz-default-plugins
sudo apt install qtbase5-dev qtdeclarative5-dev

# Build dependencies
sensor_msgs
rclcpp
geometry_msgs
rviz_common
rviz_rendering
rviz_default_plugins
pluginlib
```

## Package Structure
```
f16_hud_rviz_plugin/
├── CMakeLists.txt
├── package.xml
├── plugins_description.xml          # Plugin registration
├── src/
│   ├── f16_hud_display.cpp         # Main plugin class
│   ├── f16_hud_visual.cpp          # 3D visual rendering
│   ├── imu_processor.cpp           # IMU data processing
│   └── hud_overlay.cpp             # 2D overlay rendering
├── include/
│   └── f16_hud_rviz_plugin/
│       ├── f16_hud_display.hpp
│       ├── f16_hud_visual.hpp
│       ├── imu_processor.hpp
│       └── hud_overlay.hpp
├── ui/
│   └── f16_hud_panel.ui            # Qt UI layout
├── launch/
│   └── hud_demo.launch.py
├── config/
│   └── hud_config.rviz
└── README.md
```

## RViz2 Plugin Implementation

### Plugin Architecture
The plugin will implement two main approaches:

#### Option 1: 3D Display Plugin (Recommended)
- Inherits from `rviz_common::Display`
- Renders HUD elements as 3D objects in world space
- Allows camera interaction and perspective changes
- Better integration with RViz2's 3D environment

#### Option 2: Panel Plugin with Overlay
- Inherits from `rviz_common::Panel`
- Renders HUD as 2D overlay on viewport
- Fixed screen position regardless of camera
- Simpler implementation but less flexible

### Plugin Registration
```xml
<!-- plugins_description.xml -->
<library path="f16_hud_rviz_plugin">
  <class name="f16_hud_rviz_plugin/F16HudDisplay"
         type="f16_hud_rviz_plugin::F16HudDisplay"
         base_class_type="rviz_common::Display">
    <description>
      F-16 style HUD display showing aircraft attitude from IMU data
    </description>
  </class>
</library>
```

## HUD Display Specifications

### Visual Configuration
- **Background:** Dark transparent overlay
- **Foreground:** Bright green (RGB: 0, 255, 0)
- **Font:** Monospace, configurable size
- **Transparency:** Adjustable alpha channel
- **Scale:** Configurable size relative to viewport

### HUD Layout (Based on Authentic F-16 Reference)
```
                    30°
                ────────────
                    20°              ┌─────┐
            ────────────────────     │ 485 │
                    10°              │ 490 │
        ────────────────────────     │>495<│ 
                     0°              │ 500 │
    ┌─────┐ ═══════════════════════  │ 505 │ ┌─────┐
    │ 483 │         ══╤══            └─────┘ │5815 │ Airspeed
    │ 488 │           │                      │5820 │ Scale
    │>493<│     ──────┼──────                │>5825<│ (Simulated)
    │ 498 │           │                      │5830 │ 
    │ 503 │         ══╧══                    │5835 │
    └─────┘                                  └─────┘
        ────────────────────────     
                   -10°              
            ────────────────────     
                   -20°              
                ────────────
                   -30°

                   HDG 045°
    ────────────────────────────────────────
```

### Core HUD Elements

#### 1. Central Attitude Indicator
- **Pitch Ladder:** Horizontal lines at ±30°, ±20°, ±10°, 0°
- **Aircraft Symbol:** Fixed cross-hair in centre
- **Horizon Line:** Moves and tilts with aircraft attitude
- **Pitch Range:** ±30° visible range

#### 2. Side Information Displays
- **Left Panel:** Airspeed indicator (simulated data)
- **Right Panel:** Altitude indicator (simulated data)
- **Current values highlighted with brackets: `>495<`**

#### 3. Bottom Status Line
- **Heading:** `HDG 045°` (from IMU yaw)
- **Additional flight data as space permits**

### IMU Data Processing

#### Input Data
- **Topic:** Configurable, default `/imu/data` (sensor_msgs/Imu)
- **Required Fields:**
  - `orientation` (quaternion)
  - Update rate: 50Hz minimum

#### Coordinate Transformations
```cpp
// Convert quaternion to Euler angles
void quaternionToEuler(const geometry_msgs::msg::Quaternion& q, 
                      double& roll, double& pitch, double& yaw) {
    // Standard aerospace convention
    // Roll: rotation about X-axis (±180°)
    // Pitch: rotation about Y-axis (±90°)  
    // Yaw: rotation about Z-axis (±180°)
}
```

#### Display Mapping
- **Pitch:** Vertical movement of horizon line and pitch ladder
  - 1° pitch = configurable pixel movement
  - Pitch up = horizon moves down on screen
- **Roll:** Rotation of entire attitude display around centre
  - Bank angle displayed numerically
- **Yaw:** Heading display at bottom
  - 0-360° compass heading

### Technical Implementation

#### RViz2 Display Plugin Class
```cpp
class F16HudDisplay : public rviz_common::Display {
Q_OBJECT

public:
    F16HudDisplay();
    ~F16HudDisplay() override;

protected:
    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

private Q_SLOTS:
    void updateTopic();
    void updateScale();
    void updateTransparency();

private:
    void processMessage(const sensor_msgs::msg::Imu::SharedPtr msg);
    void createVisuals();
    void updateVisuals();
    
    // Properties
    rviz_common::properties::RosTopicProperty* topic_property_;
    rviz_common::properties::FloatProperty* scale_property_;
    rviz_common::properties::FloatProperty* transparency_property_;
    rviz_common::properties::ColorProperty* colour_property_;
    
    // ROS2 subscription
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // Visual elements
    std::unique_ptr<F16HudVisual> hud_visual_;
    
    // Current state
    double current_roll_ = 0.0;
    double current_pitch_ = 0.0;
    double current_yaw_ = 0.0;
    
    // Threading
    std::mutex data_mutex_;
};
```

#### Visual Rendering Class
```cpp
class F16HudVisual {
public:
    F16HudVisual(Ogre::SceneManager* scene_manager, 
                 Ogre::SceneNode* parent_node);
    ~F16HudVisual();
    
    void setAttitude(double roll, double pitch, double yaw);
    void setScale(float scale);
    void setTransparency(float alpha);
    void setColour(const Ogre::ColourValue& colour);
    
private:
    void createPitchLadder();
    void createAircraftSymbol();
    void createSideTapes();
    void createHeadingDisplay();
    
    void updatePitchLadder();
    void updateAircraftSymbol();
    void updateSideTapes();
    void updateHeadingDisplay();
    
    // Ogre rendering objects
    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneNode* pitch_ladder_node_;
    Ogre::SceneNode* aircraft_symbol_node_;
    
    // HUD elements
    std::vector<Ogre::ManualObject*> pitch_lines_;
    std::vector<Ogre::MovableText*> pitch_labels_;
    Ogre::ManualObject* aircraft_symbol_;
    Ogre::ManualObject* airspeed_tape_;
    Ogre::ManualObject* altitude_tape_;
    
    // Current state
    double roll_, pitch_, yaw_;
    float scale_;
    float transparency_;
    Ogre::ColourValue colour_;
};
```

#### Plugin Properties Panel
```cpp
// Properties that appear in RViz2 left panel
topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "/imu/data",
    QString::fromStdString(rosidl_generator_traits::data_type<sensor_msgs::msg::Imu>()),
    "IMU topic to subscribe to",
    this, SLOT(updateTopic()));

scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 1.0f,
    "Scale factor for HUD elements",
    this, SLOT(updateScale()));

transparency_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.8f,
    "Transparency of HUD elements",
    this, SLOT(updateTransparency()));
```

### CMakeLists.txt Configuration
```cmake
cmake_minimum_required(VERSION 3.5)
project(f16_hud_rviz_plugin)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)

# Qt5 components
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/f16_hud_display.cpp
  src/f16_hud_visual.cpp
  src/imu_processor.cpp
)

# Create plugin library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

# Plugin registration
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Development Phases

#### Phase 1: Plugin Infrastructure
- [ ] Create ROS2 package skeleton with RViz2 plugin support
- [ ] Implement basic Display plugin class
- [ ] Set up Ogre3D rendering pipeline
- [ ] Create plugin registration and CMakeLists.txt

#### Phase 2: Basic HUD Elements
- [ ] Implement fixed aircraft symbol rendering
- [ ] Create pitch ladder geometry
- [ ] Add basic text rendering for degree markings
- [ ] Implement IMU subscription and data processing

#### Phase 3: Dynamic Behaviour
- [ ] Connect IMU data to pitch ladder movement
- [ ] Implement roll rotation of attitude display
- [ ] Add heading display updates
- [ ] Create side information panels

#### Phase 4: Plugin Integration
- [ ] Add configurable properties panel
- [ ] Implement proper plugin lifecycle management
- [ ] Add configuration save/load support
- [ ] Create example RViz2 configuration

#### Phase 5: Polish and Documentation
- [ ] Optimise rendering performance
- [ ] Add comprehensive error handling
- [ ] Create launch files and documentation
- [ ] Performance testing and validation

### Plugin Configuration
```yaml
# Example RViz2 config snippet
Displays:
  - Class: f16_hud_rviz_plugin/F16HudDisplay
    Enabled: true
    Name: F16_HUD
    Topic: /imu/data
    Scale: 1.0
    Alpha: 0.8
    Colour: [0, 255, 0]
```

### Testing Strategy
1. **Unit Tests:** Individual component testing
2. **Integration Tests:** Plugin loading and RViz2 integration
3. **Performance Tests:** Rendering performance in RViz2
4. **Usability Tests:** User interface and configuration testing

### Advantages of RViz2 Plugin Approach
- **Integration:** Seamless integration with existing ROS2 workflows
- **Flexibility:** Users can combine HUD with other visualisations
- **Configuration:** Built-in save/load of display configurations
- **3D Context:** Can show HUD in relation to robot/world models
- **Extensibility:** Easy to add more flight instruments as separate plugins

### Success Criteria
- [ ] Plugin loads successfully in RViz2
- [ ] Displays authentic F-16 style HUD symbology
- [ ] Real-time response to IMU orientation changes
- [ ] Configurable properties through RViz2 interface
- [ ] Stable operation with other RViz2 plugins
- [ ] Professional appearance matching reference image
- [ ] Comprehensive documentation and examples