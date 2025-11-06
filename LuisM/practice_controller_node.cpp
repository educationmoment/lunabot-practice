#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "SparkMax.hpp"
#include <cmath>
#include <algorithm>
#include <string>

// Motor configuration
const int LEFT_ID  = 1;
const int RIGHT_ID = 2;
const float DEADZONE = 0.1f;

// Gamepad axis mapping (based on Xbox-style layout)
namespace Gamepad
{
    enum Axes
    {
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 2,
        RIGHT_Y = 3
    };

    enum Buttons
    {
        A = 0,
        B = 1,
        X = 2,
        Y = 3,
        LB = 4,
        RB = 5,
        LT = 6,
        RT = 7
    };
}

// Helper functions
float applyDeadzone(float value, float threshold = DEADZONE)
{
    if (std::fabs(value) < threshold)
        return 0.0f;
    return value;
}

float clamp(float value, float minVal, float maxVal)
{
    return std::max(minVal, std::min(maxVal, value));
}

// Main Controller Node Class
class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(const std::string &can_iface)
    : Node("controller_node"),
      left_motor_(can_iface, LEFT_ID),
      right_motor_(can_iface, RIGHT_ID)
    {
        RCLCPP_INFO(get_logger(), "Controller node started on interface: %s", can_iface.c_str());

    
        left_motor_.SetIdleMode(IdleMode::kBrake);
        right_motor_.SetIdleMode(IdleMode::kBrake);
        left_motor_.SetMotorType(MotorType::kBrushless);
        right_motor_.SetMotorType(MotorType::kBrushless);

 
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Waiting for /joy messages...");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        if (msg->axes.size() < 4)
        {
            stopMotors();
            RCLCPP_WARN(get_logger(), "Invalid joystick message. Stopping motors.");
            return;
        }

        float left_input  = -msg->axes[Gamepad::LEFT_Y];
        float right_input = -msg->axes[Gamepad::RIGHT_Y];

        left_input  = applyDeadzone(left_input);
        right_input = applyDeadzone(right_input);

        bool half_speed = (msg->buttons.size() > Gamepad::LB) && (msg->buttons[Gamepad::LB] == 1);
        float speed_scale = half_speed ? 0.5f : 1.0f;

        left_input  = clamp(left_input * speed_scale, -1.0f, 1.0f);
        right_input = clamp(right_input * speed_scale, -1.0f, 1.0f);

        left_motor_.SetDutyCycle(left_input);
        right_motor_.SetDutyCycle(right_input);

        left_motor_.HeartBeat();
        right_motor_.HeartBeat();
    }

    void stopMotors()
    {
        left_motor_.SetDutyCycle(0.0f);
        right_motor_.SetDutyCycle(0.0f);
        left_motor_.HeartBeat();
        right_motor_.HeartBeat();
    }

    SparkMax left_motor_;
    SparkMax right_motor_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

// Main Function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string can_interface = "can0"; 
    auto temp_node = rclcpp::Node::make_shared("param_loader");
    temp_node->declare_parameter<std::string>("can_interface", "can0");
    temp_node->get_parameter("can_interface", can_interface);

    auto node = std::make_shared<ControllerNode>(can_interface);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
