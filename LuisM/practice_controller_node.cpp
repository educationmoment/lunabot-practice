#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "SparkMax.hpp"
#include <cmath>
#include <algorithm>
#include <string>

//Can IDs
const int LEFT_ID = 1;
const int RIGHT_ID = 2;
const int LEFT_LIFT = 3;
const int RIGHT_LIFT = 4;
const int TILT = 5;

const float DEADZONE = 0.1f;

// Gamepad axis & button mapping (Xbox)
namespace Gp
{
    enum Buttons
    {
        _A = 0,
        _B = 1,
        _X = 2,
        _Y = 3,
        _LEFT_BUMPER = 4,
        _RIGHT_BUMPER = 5,
        _LEFT_TRIGGER = 6,
        _RIGHT_TRIGGER = 7,
        _WINDOW_KEY = 8,
        _D_PAD_UP = 12,
        _D_PAD_DOWN = 13,
        _D_PAD_LEFT = 14,
        _D_PAD_RIGHT = 15,
        _X_BOX_KEY = 16
    };

    enum Axes
    {
        _LEFT_HORIZONTAL_STICK = 0,
        _LEFT_VERTICAL_STICK = 1,
        _RIGHT_HORIZONTAL_STICK = 2,
        _RIGHT_VERTICAL_STICK = 3,
    };
}

// Helper functions
float applyDeadzone(float value, float threshold = DEADZONE)
{
    return (std::fabs(value) < threshold) ? 0.0f : value;
}

float clamp(float value, float minVal, float maxVal)
{
    return std::max(minVal, std::min(maxVal, value));
}

// Main Controller Node Class
class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(const std::string &can_interface)
        : Node("controller_node"),
          left_motor_(can_interface, LEFT_ID),
          right_motor_(can_interface, RIGHT_ID),
          left_lift_(can_interface, LEFT_LIFT),
          right_lift_(can_interface, RIGHT_LIFT),
          tilt_motor_(can_interface, TILT)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Drivetrain Motors...");

        // DRIVETRAIN
        left_motor_.SetIdleMode(IdleMode::kBrake);
        right_motor_.SetIdleMode(IdleMode::kBrake);

        left_motor_.SetMotorType(MotorType::kBrushless);
        right_motor_.SetMotorType(MotorType::kBrushless);

        left_motor_.SetSensorType(SensorType::kHallSensor);
        right_motor_.SetSensorType(SensorType::kHallSensor);

        left_motor_.SetInverted(false);
        right_motor_.SetInverted(true);

        left_motor_.SetP(0, 0.0002f);
        left_motor_.SetI(0, 0.0f);
        left_motor_.SetD(0, 0.0f);
        left_motor_.SetF(0, 0.00021f);

        right_motor_.SetP(0, 0.0002f);
        right_motor_.SetI(0, 0.0f);
        right_motor_.SetD(0, 0.0f);
        right_motor_.SetF(0, 0.00021f);

        left_motor_.BurnFlash();
        right_motor_.BurnFlash();

        RCLCPP_INFO(this->get_logger(), "Initializing Actuators...");

        // LIFT ACTUATORS
        left_lift_.SetIdleMode(IdleMode::kBrake);
        right_lift_.SetIdleMode(IdleMode::kBrake);

        left_lift_.SetMotorType(MotorType::kBrushed);
        right_lift_.SetMotorType(MotorType::kBrushed);

        left_lift_.SetSensorType(SensorType::kEncoder);
        right_lift_.SetSensorType(SensorType::kEncoder);

        left_lift_.SetInverted(true);
        right_lift_.SetInverted(true);

        left_lift_.SetP(0, 1.5f);
        right_lift_.SetP(0, 1.5f);

        left_lift_.BurnFlash();
        right_lift_.BurnFlash();

        // TILT ACTUATOR
        tilt_motor_.SetIdleMode(IdleMode::kBrake);
        tilt_motor_.SetMotorType(MotorType::kBrushed);
        tilt_motor_.SetSensorType(SensorType::kEncoder);
        tilt_motor_.SetInverted(true);
        tilt_motor_.SetP(0, 1.5f);
        tilt_motor_.BurnFlash();

        RCLCPP_INFO(this->get_logger(), "Motors & Actuators Initialized.");

        // ROS Joy Subscription
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Joystick Subscription Initialized.");
    }

private:

    // Joystick Callback
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Safety check
        if (msg->axes.size() < 4)
        {
            stopMotors();
            RCLCPP_WARN(this->get_logger(), "Invalid joystick message. Stopping motors.");
            return;
        }

        // DRIVE
        float left_input  = -msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
        float right_input = -msg->axes[Gp::Axes::_RIGHT_VERTICAL_STICK];

        left_input  = applyDeadzone(left_input);
        right_input = applyDeadzone(right_input);

        bool half_speed =
            (msg->buttons.size() > Gp::Buttons::_LEFT_BUMPER) &&
            (msg->buttons[Gp::Buttons::_LEFT_BUMPER] == 1);

        float speed_scale = half_speed ? 0.5f : 1.0f;

        left_input  = clamp(left_input * speed_scale,  -1.0f, 1.0f);
        right_input = clamp(right_input * speed_scale, -1.0f, 1.0f);

        left_motor_.SetDutyCycle(left_input);
        right_motor_.SetDutyCycle(right_input);

        left_motor_.HeartBeat();
        right_motor_.HeartBeat();

        // LIFT ACTUATORS
        float lift_power = 0.0f;

        if (msg->buttons[Gp::Buttons::_D_PAD_UP] == 1)
            lift_power = +1.0f;
        else if (msg->buttons[Gp::Buttons::_D_PAD_DOWN] == 1)
            lift_power = -1.0f;

        left_lift_.SetDutyCycle(lift_power);
        right_lift_.SetDutyCycle(lift_power);

        left_lift_.HeartBeat();
        right_lift_.HeartBeat();

        // TILT ACTUATOR
        float tilt_power = 0.0f;

        if (msg->buttons[Gp::Buttons::_D_PAD_LEFT] == 1)
            tilt_power = +1.0f;
        else if (msg->buttons[Gp::Buttons::_D_PAD_RIGHT] == 1)
            tilt_power = -1.0f;

        tilt_motor_.SetDutyCycle(tilt_power);
        tilt_motor_.HeartBeat();
    }

    // Stop motors as a safety measure
    void stopMotors()
    {
        left_motor_.SetDutyCycle(0.0f);
        right_motor_.SetDutyCycle(0.0f);
        left_motor_.HeartBeat();
        right_motor_.HeartBeat();
    }

// Objectives
    SparkMax left_motor_;
    SparkMax right_motor_;

    SparkMax left_lift_;
    SparkMax right_lift_;

    SparkMax tilt_motor_;

    // ROS subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
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
