#include "f16_hud_rviz_plugin/f16_hud_display.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>

namespace f16_hud_rviz_plugin
{

F16HudDisplay::F16HudDisplay()
    : _color(0.0f, 1.0f, 0.0f, 0.8f)  // Default green color
{
    _topicProperty = new rviz_common::properties::RosTopicProperty(
        "Topic", "/imu/data",
        QString::fromStdString(rosidl_generator_traits::data_type<sensor_msgs::msg::Imu>()),
        "IMU topic to subscribe to",
        this, SLOT(updateTopic()));

    _scaleProperty = new rviz_common::properties::FloatProperty(
        "Scale", 1.0f,
        "Scale factor for HUD elements",
        this, SLOT(updateScale()));
    _scaleProperty->setMin(0.1f);
    _scaleProperty->setMax(5.0f);

    _transparencyProperty = new rviz_common::properties::FloatProperty(
        "Alpha", 0.8f,
        "Transparency of HUD elements",
        this, SLOT(updateTransparency()));
    _transparencyProperty->setMin(0.0f);
    _transparencyProperty->setMax(1.0f);

    _colorProperty = new rviz_common::properties::ColorProperty(
        "Color", QColor(0, 255, 0),
        "Color of HUD elements",
        this, SLOT(updateColor()));
}

F16HudDisplay::~F16HudDisplay()
{
    reset();
}

void F16HudDisplay::onInitialize()
{
    _imuProcessor = std::make_unique<ImuProcessor>();
    _color = Ogre::ColourValue(0.0f, 1.0f, 0.0f, _transparency);
}

void F16HudDisplay::onEnable()
{
    createVisuals();
    updateTopic();
}

void F16HudDisplay::onDisable()
{
    _imuSubscription.reset();
    if (_hudVisual)
    {
        _hudVisual->setVisible(false);
    }
}

void F16HudDisplay::reset()
{
    Display::reset();
    _imuSubscription.reset();
    _hudVisual.reset();
}

void F16HudDisplay::update(float wall_dt, float ros_dt)
{
    (void)wall_dt;
    (void)ros_dt;
    updateVisuals();
}

void F16HudDisplay::updateTopic()
{
    _imuSubscription.reset();
    
    if (!isEnabled())
    {
        return;
    }

    try
    {
        _imuSubscription = context_->getRosNodeAbstraction().lock()->get_raw_node()->
            create_subscription<sensor_msgs::msg::Imu>(
                _topicProperty->getTopicStd(),
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                    processMessage(msg);
                });
        setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    }
    catch (rclcpp::exceptions::InvalidTopicNameError& e)
    {
        setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                 QString("Error subscribing: ") + e.what());
    }
}

void F16HudDisplay::updateScale()
{
    _scale = _scaleProperty->getFloat();
    if (_hudVisual)
    {
        _hudVisual->setScale(_scale);
    }
}

void F16HudDisplay::updateTransparency()
{
    _transparency = _transparencyProperty->getFloat();
    _color.a = _transparency;
    if (_hudVisual)
    {
        _hudVisual->setTransparency(_transparency);
    }
}

void F16HudDisplay::updateColor()
{
    QColor qtColor = _colorProperty->getColor();
    _color.r = qtColor.redF();
    _color.g = qtColor.greenF();
    _color.b = qtColor.blueF();
    _color.a = _transparency;
    
    if (_hudVisual)
    {
        _hudVisual->setColor(_color);
    }
}

void F16HudDisplay::processMessage(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!_imuProcessor->isValidQuaternion(msg->orientation))
    {
        setStatus(rviz_common::properties::StatusProperty::Warn, "IMU",
                 "Invalid quaternion data received");
        return;
    }

    std::lock_guard<std::mutex> lock(_dataMutex);
    
    _imuProcessor->quaternionToEuler(msg->orientation, 
                                   _currentRoll, _currentPitch, _currentYaw);
    
    setStatus(rviz_common::properties::StatusProperty::Ok, "IMU", "OK");
}

void F16HudDisplay::createVisuals()
{
    if (!scene_manager_ || !scene_node_)
    {
        return;
    }

    _hudVisual = std::make_unique<F16HudVisual>(scene_manager_, scene_node_);
    _hudVisual->setScale(_scale);
    _hudVisual->setTransparency(_transparency);
    _hudVisual->setColor(_color);
    _hudVisual->setVisible(true);
}

void F16HudDisplay::updateVisuals()
{
    if (!_hudVisual)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(_dataMutex);
    _hudVisual->setAttitude(_currentRoll, _currentPitch, _currentYaw);
}

}  // namespace f16_hud_rviz_plugin

PLUGINLIB_EXPORT_CLASS(f16_hud_rviz_plugin::F16HudDisplay, rviz_common::Display)