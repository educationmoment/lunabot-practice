#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0;  // rpm, after gearbox turns into 11.1 RPM

enum CAN_IDs  // assign ID to each motor
{
  LEFT_MOTOR = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT = 3,
  RIGHT_LIFT = 4,
};

namespace Controller // corresponding numbers are indicies
{
  enum Buttons
  {
    _A = 0,             // Excavation Autonomy
    _B = 1,             // Stop Automation
    _X = 2,             // Excavation Reset
    _Y = 3,             // Deposit Autonomy
    _LEFT_BUMPER = 4,   // Alternate between control modes
    _RIGHT_BUMPER = 5,  // Vibration Toggle
    _LEFT_TRIGGER = 6,  // Safety Trigger
    _RIGHT_TRIGGER = 7, // Safety Trigger
    _WINDOW_KEY = 8,    // Button 8 /** I do not know what else to call this key */
    _D_PAD_UP = 12,     // Lift Actuator UP
    _D_PAD_DOWN = 13,   // Lift Actuator DOWN
    _D_PAD_LEFT = 14,   // Tilt Actuator Up   /** CHECK THESE */
    _D_PAD_RIGHT = 15,  // Tilt Actuator Down /** CHECK THESE */
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

class ControllerNode: public rclcpp::Node  // ControllerNode inherits rclcpp::Node (ROS node) features
{
  public:
    ControllerNode(const std::string &can_interface) : Node("controller_node")
    {
      // Initialize motors and lift
      leftMotor = SparkMax(can_interface, LEFT_MOTOR);
      rightMotor = SparkMax(can_interface, RIGHT_MOTOR);
      leftLift = SparkMax(can_interface, LEFT_LIFT);
      rightLift = SparkMax(can_interface, RIGHT_LIFT);

      RCLCPP_INFO(this->get_logger(), "Begin Initializing Node");
      RCLCPP_INFO(this->get_logger(), "Initializing Motor Controllers");

      leftMotor.SetIdleMode(IdleMode::kBrake);
      rightMotor.SetIdleMode(IdleMode::kBrake);
      leftMotor.SetMotorType(MotorType::kBrushless);
      rightMotor.SetMotorType(MotorType::kBrushless);
      leftMotor.SetSensorType(SensorType::kHallSensor);
      rightMotor.SetSensorType(SensorType::kHallSensor);
      // Initializes the settings for the drivetrain motors

      leftLift.SetIdleMode(IdleMode::kBrake);
      rightLift.SetIdleMode(IdleMode::kBrake);
      leftLift.SetMotorType(MotorType::kBrushed);
      rightLift.SetMotorType(MotorType::kBrushed);
      leftLift.SetSensorType(SensorType::kEncoder);
      rightLift.SetSensorType(SensorType::kEncoder);
      // Initializes the settings for the lift actuators

      leftMotor.SetInverted(false);
      rightMotor.SetInverted(true);
      leftLift.SetInverted(true);
      rightLift.SetInverted(true);
      // Initializes the inverting status

      leftMotor.SetP(0, 0.0002f);
      leftMotor.SetI(0, 0.0f);
      leftMotor.SetD(0, 0.0f);
      leftMotor.SetF(0, 0.00021f);
      // PID settings for left motor

      rightMotor.SetP(0, 0.0002f);
      rightMotor.SetI(0, 0.0f);
      rightMotor.SetD(0, 0.0f);
      rightMotor.SetF(0, 0.00021f);
      // PID settings for right motor

      leftLift.SetP(0, 1.51f);
      leftLift.SetI(0, 0.0f);
      leftLift.SetD(0, 0.0f);
      leftLift.SetF(0, 0.00021f);
      // PID settings for left lift

      rightLift.SetP(0, 1.51f);
      rightLift.SetI(0, 0.0f);
      rightLift.SetD(0, 0.0f);
      rightLift.SetF(0, 0.00021f);
      // PID settings for right lift

      leftMotor.BurnFlash();
      rightMotor.BurnFlash();
      leftLift.BurnFlash();
      rightLift.BurnFlash();

      RCLCPP_INFO(this->get_logger(), "Motor Controllers Initialized");

      // ROS subscriptions
      // might need to move this
      RCLCPP_INFO(this->get_logger(), "Initializing Joy Subscription");
      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Joy Subscription Initialized");
    }
  private:
    // Direct object members.
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMax leftLift;
    SparkMax rightLift;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    // float left_lift_position = 0.0f;
    // float right_lift_position = 0.0f;

    float computeStepOutput(float value)
    {
      float absVal = std::fabs(value);
      const float thresholds[] = {0.25f, 0.5f, 0.75f, 1.0f};
      const float outputs[] = {0.0f, 0.25f, 0.5f, 0.75f};

      // can use .size() instead of 4
      for (int i = 0; i < 4; ++i) {
        if (absVal < thresholds[i])
          return (value > 0 ? outputs[i] : -outputs[i]);
      }

      return (value > 0 ? 1.0f : -1.0f);
    }


    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
      float left_drive = 0.0f;
      float right_drive= 0.0f;
      float left_drive_raw = 0.0f;
      float right_drive_raw = 0.0f;
      float lift_trigger = 0.0f;
      float lift = 0.0f;

      leftMotor.Heartbeat();
      rightMotor.Heartbeat();
      leftLift.Heartbeat();
      rightLift.Heartbeat();

      // take in input
      // use left trigger for lift
      float leftJS = joy_msg->axes[Controller::Axes::_LEFT_VERTICAL_STICK];
      float rightJS = joy_msg->axes[Controller::Axes::_RIGHT_VERTICAL_STICK];
      lift_trigger = joy_msg->axes[Controller::Axes::_LEFT_TRIGGER];

      // make sure drive input is in between -1 and 1
      left_drive_raw = std::max(-1.0f, std::min(1.0f, leftJS));
      right_drive_raw = std::max(-1.0f, std::min(1.0f, rightJS));

      // make sure lift input is in between 0 and 1
      // lift = std::max(0.0f, std::min(1.0f, rightJS));

      //process drive input
      left_drive = computeStepOutput(left_drive_raw);
      right_drive = computeStepOutput(right_drive_raw);

      // move wheels
      leftMotor.SetVelocity(left_drive);
      rightMotor.SetVelocity(right_drive);

      // perform lift
      // do i use SetPosition?
      leftLift.SetPosition(lift_trigger);
      rightLift.SetPosition(lift_trigger);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Left Motor: %.2f | Right Motor: %.2f | Lift: %.2f",
        left_drive, right_drive, lift);
    }
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  std::string can_interface = "can0";
  auto temp_node = std::make_shared<ControllerNode>(can_interface);
  RCLCPP_INFO(temp_node->get_logger(), "started controller node");
  rclcpp::spin(temp_node);
  rclcpp::shutdown();
  return 0;
}
