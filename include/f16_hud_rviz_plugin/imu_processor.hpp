#pragma once

#include <geometry_msgs/msg/quaternion.hpp>

namespace f16_hud_rviz_plugin
{

class ImuProcessor
{
public:
    ImuProcessor() = default;
    ~ImuProcessor() = default;
    
    void quaternionToEuler(const geometry_msgs::msg::Quaternion& quaternion,
                          double& roll, double& pitch, double& yaw) const;
    
    bool isValidQuaternion(const geometry_msgs::msg::Quaternion& quaternion) const;
    
private:
    static constexpr double QUATERNION_TOLERANCE = 1e-6;
};

}  // namespace f16_hud_rviz_plugin