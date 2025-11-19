#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0;  // rpm, after gearbox turns into 11.1 RPM

enum CAN_IDs
{
  LEFT_MOTOR = 1,
  RIGHT_MOTOR = 2
};

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

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(const std::string &can_interface) : Node("controller_node")
  {
    // Initialize motors
    leftMotor = SparkMax(can_interface, LEFT_MOTOR);
    rightMotor = SparkMax(can_interface, RIGHT_MOTOR);
    leftLift = SparkMax(can_interface, LEFT_MOTOR + 2);
    rightLift = SparkMax(can_interface, RIGHT_MOTOR + 2);

    RCLCPP_INFO(this->get_logger(), "Initializing Motor Controllers");

    leftMotor.SetIdleMode(IdleMode::kBrake);
    rightMotor.SetIdleMode(IdleMode::kBrake);
    leftMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetSensorType(SensorType::kHallSensor);
    rightMotor.SetSensorType(SensorType::kHallSensor);

    leftMotor.SetInverted(false);
    rightMotor.SetInverted(true);

    leftLift.SetIdleMode(IdleMode::kBrake);
    rightLift.SetIdleMode(IdleMode::kBrake);
    leftLift.SetMotorType(MotorType::kBrushed);
    rightLift.SetMotorType(MotorType::kBrushed);
    leftLift.SetSensorType(SensorType::kEncoder);
    rightLift.SetSensorType(SensorType::kEncoder);

    leftMotor.SetP(0, 0.0002f);
    leftMotor.SetI(0, 0.0f);
    leftMotor.SetD(0, 0.0f);
    leftMotor.SetF(0, 0.00021f);

    rightMotor.SetP(0, 0.0002f);
    rightMotor.SetI(0, 0.0f);
    rightMotor.SetD(0, 0.0f);
    rightMotor.SetF(0, 0.00021f);

    leftMotor.BurnFlash();
    rightMotor.BurnFlash();
    leftLift.BurnFlash();
    rightLift.BurnFlash();

    RCLCPP_INFO(this->get_logger(), "Motor Controllers Initialized");

    RCLCPP_INFO(this->get_logger(), "Initializing Joy Subscription");
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Joy Subscription Initialized");
  }

private:
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  float computeStepOutput(float value)
  {
    // [-1, 1]
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    //   steps: 0, 0.25, 0.5, 0.75, 1.0
    float stepSize = 0.25f;
    float sign = (value >= 0.0f) ? 1.0f : -1.0f;
    float absVal = std::fabs(value);

    // Round to nearest step
    int step = static_cast<int>(std::round(absVal / stepSize));
    return step * stepSize * sign;
  }



  // JOY CALLBACK
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    float left_joystick = -joy_msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
    float right_joystick = -joy_msg->axes[Gp::Axes::_RIGHT_VERTICAL_STICK];
    float lift_trigger = joy_msg->axes[Gp::Axes::_LEFT_TRIGGER]; 

    // ------------ LEFT MOTOR ------------
    float left_processed = 0.0f;
    if (left_joystick > 1.0f)
        left_processed = 1.0f;
    else if (left_joystick < -1.0f)
        left_processed = -1.0f;
    else
        left_processed = left_joystick;

    left_processed = computeStepOutput(left_processed);

    // ------------ RIGHT MOTOR ------------
    float right_processed = 0.0f;
    if (right_joystick > 1.0f)
        right_processed = 1.0f;
    else if (right_joystick < -1.0f)
        right_processed = -1.0f;
    else
        right_processed = right_joystick;

    right_processed = computeStepOutput(right_processed);

    // Set drive motors
    leftMotor.SetDutyCycle(left_processed);
    rightMotor.SetDutyCycle(right_processed);

    // --- Lift control-left trigger ---
    float lift_duty = (lift_trigger + 1.0f) / 2.0f;
    lift_duty = std::clamp(lift_duty, 0.0f, 1.0f);
    leftLift.SetDutyCycle(lift_duty);
    rightLift.SetDutyCycle(lift_duty);


    leftMotor.Heartbeat();
    rightMotor.Heartbeat();
    leftLift.Heartbeat();
    rightLift.Heartbeat();
  }

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::string can_interface = "can0";
  auto temp_node = rclcpp::Node::make_shared("controller_param_node");
  temp_node->declare_parameter<std::string>("can_interface", "can0");
  temp_node->get_parameter("can_interface", can_interface);
  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
