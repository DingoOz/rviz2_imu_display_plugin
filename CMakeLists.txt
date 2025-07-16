cmake_minimum_required(VERSION 3.5)
project(f16_hud_rviz_plugin)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# Headers with Q_OBJECT that need MOC processing
set(HEADERS_TO_MOC
  include/f16_hud_rviz_plugin/f16_hud_display.hpp
)

# Source files
set(SOURCES
  src/f16_hud_display.cpp
  src/f16_hud_visual.cpp
  src/imu_processor.cpp
)

# Process headers with MOC
qt5_wrap_cpp(MOC_SOURCES ${HEADERS_TO_MOC})

# Create plugin library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${MOC_SOURCES})

# Test IMU publisher executable
add_executable(imu_test_publisher src/imu_test_publisher.cpp)

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

# Link dependencies for test publisher
ament_target_dependencies(imu_test_publisher
  rclcpp
  sensor_msgs
  tf2
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

install(TARGETS imu_test_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  rviz_common
  rviz_rendering
  pluginlib
)

ament_package()