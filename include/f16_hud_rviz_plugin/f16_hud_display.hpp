#pragma once

#include <memory>
#include <mutex>

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include "f16_hud_rviz_plugin/f16_hud_visual.hpp"
#include "f16_hud_rviz_plugin/imu_processor.hpp"

namespace f16_hud_rviz_plugin
{

class F16HudDisplay : public rviz_common::Display
{
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
    void updateColor();

private:
    void processMessage(const sensor_msgs::msg::Imu::SharedPtr msg);
    void createVisuals();
    void updateVisuals();
    
    // Properties
    rviz_common::properties::RosTopicProperty* _topicProperty;
    rviz_common::properties::FloatProperty* _scaleProperty;
    rviz_common::properties::FloatProperty* _transparencyProperty;
    rviz_common::properties::ColorProperty* _colorProperty;
    
    // ROS2 subscription
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSubscription;
    
    // Visual elements
    std::unique_ptr<F16HudVisual> _hudVisual;
    
    // IMU processing
    std::unique_ptr<ImuProcessor> _imuProcessor;
    
    // Current state
    double _currentRoll = 0.0;
    double _currentPitch = 0.0;
    double _currentYaw = 0.0;
    
    // Threading
    std::mutex _dataMutex;
    
    // Configuration
    float _scale = 1.0f;
    float _transparency = 0.8f;
    Ogre::ColourValue _color;
};

}  // namespace f16_hud_rviz_plugin