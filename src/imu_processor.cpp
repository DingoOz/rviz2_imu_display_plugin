#include "f16_hud_rviz_plugin/imu_processor.hpp"

#include <cmath>

namespace f16_hud_rviz_plugin
{

void ImuProcessor::quaternionToEuler(const geometry_msgs::msg::Quaternion& quaternion,
                                    double& roll, double& pitch, double& yaw) const
{
    // Extract quaternion components
    double x = quaternion.x;
    double y = quaternion.y;
    double z = quaternion.z;
    double w = quaternion.w;
    
    // Convert quaternion to Euler angles using aerospace convention
    // Roll (x-axis rotation)
    double sinRollCosYaw = 2.0 * (w * x + y * z);
    double cosRollCosYaw = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinRollCosYaw, cosRollCosYaw);
    
    // Pitch (y-axis rotation)
    double sinPitch = 2.0 * (w * y - z * x);
    if (std::abs(sinPitch) >= 1.0)
    {
        pitch = std::copysign(M_PI / 2.0, sinPitch);  // Use 90 degrees if out of range
    }
    else
    {
        pitch = std::asin(sinPitch);
    }
    
    // Yaw (z-axis rotation)
    double sinYawCosYaw = 2.0 * (w * z + x * y);
    double cosYawCosYaw = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(sinYawCosYaw, cosYawCosYaw);
}

bool ImuProcessor::isValidQuaternion(const geometry_msgs::msg::Quaternion& quaternion) const
{
    // Check if quaternion is normalized (magnitude should be 1)
    double magnitude = std::sqrt(
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z +
        quaternion.w * quaternion.w
    );
    
    // Check if magnitude is close to 1 within tolerance
    if (std::abs(magnitude - 1.0) > QUATERNION_TOLERANCE)
    {
        return false;
    }
    
    // Check for NaN values
    if (std::isnan(quaternion.x) || std::isnan(quaternion.y) ||
        std::isnan(quaternion.z) || std::isnan(quaternion.w))
    {
        return false;
    }
    
    // Check for infinite values
    if (std::isinf(quaternion.x) || std::isinf(quaternion.y) ||
        std::isinf(quaternion.z) || std::isinf(quaternion.w))
    {
        return false;
    }
    
    return true;
}

}  // namespace f16_hud_rviz_plugin