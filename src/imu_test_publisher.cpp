#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

class ImuTestPublisher : public rclcpp::Node
{
public:
    ImuTestPublisher() : Node("imu_test_publisher"), _time(0.0)
    {
        _publisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        
        // Get parameters
        this->declare_parameter("publish_rate", 50.0);
        double publishRate = this->get_parameter("publish_rate").as_double();
        
        // Create timer
        auto timerPeriod = std::chrono::milliseconds(static_cast<int>(1000.0 / publishRate));
        _timer = this->create_wall_timer(timerPeriod, 
            std::bind(&ImuTestPublisher::publishImuData, this));
        
        RCLCPP_INFO(this->get_logger(), "IMU Test Publisher started at %.1f Hz", publishRate);
        RCLCPP_INFO(this->get_logger(), "Publishing synthetic F-16 flight data to /imu/data");
    }

private:
    void publishImuData()
    {
        auto msg = sensor_msgs::msg::Imu();
        
        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        
        // Generate synthetic flight data patterns
        double roll, pitch, yaw;
        generateFlightPattern(roll, pitch, yaw);
        
        // Convert to quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        
        msg.orientation.x = quaternion.x();
        msg.orientation.y = quaternion.y();
        msg.orientation.z = quaternion.z();
        msg.orientation.w = quaternion.w();
        
        // Set covariance (not used by HUD but good practice)
        msg.orientation_covariance[0] = 0.01;
        msg.orientation_covariance[4] = 0.01;
        msg.orientation_covariance[8] = 0.01;
        
        _publisher->publish(msg);
        
        // Log current attitude every 2 seconds
        if (static_cast<int>(_time * 10) % 20 == 0)
        {
            RCLCPP_INFO(this->get_logger(), 
                "Attitude: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        }
        
        _time += 0.02;  // Assuming 50Hz
    }
    
    void generateFlightPattern(double& roll, double& pitch, double& yaw)
    {
        // Generate realistic F-16 flight maneuvers
        double t = _time;
        
        if (t < 10.0)
        {
            // Level flight with small oscillations
            roll = 0.05 * std::sin(t * 0.5);
            pitch = 0.02 * std::cos(t * 0.3);
            yaw = t * 0.1;  // Gentle turn
        }
        else if (t < 20.0)
        {
            // Banking turn
            double bankTime = t - 10.0;
            roll = 0.5 * std::sin(bankTime * 0.3);  // 30° bank
            pitch = 0.1 * std::sin(bankTime * 0.2);
            yaw = (t - 10.0) * 0.2;
        }
        else if (t < 30.0)
        {
            // Climb maneuver
            double climbTime = t - 20.0;
            roll = 0.1 * std::sin(climbTime * 0.4);
            pitch = 0.3 * std::sin(climbTime * 0.2);  // 15° pitch up
            yaw = 20.0 * 0.2 + climbTime * 0.05;
        }
        else if (t < 40.0)
        {
            // Barrel roll maneuver
            double rollTime = t - 30.0;
            roll = 2.0 * rollTime * 0.3;  // Complete roll
            pitch = 0.2 * std::sin(rollTime * 1.2);
            yaw = 30.0 * 0.2 + 10.0 * 0.05 + rollTime * 0.1;
        }
        else
        {
            // Reset and repeat
            _time = 0.0;
            roll = pitch = yaw = 0.0;
        }
        
        // Clamp values to realistic ranges
        roll = std::clamp(roll, -M_PI, M_PI);
        pitch = std::clamp(pitch, -M_PI/2, M_PI/2);
        yaw = std::fmod(yaw, 2.0 * M_PI);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    double _time;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuTestPublisher>());
    rclcpp::shutdown();
    return 0;
}