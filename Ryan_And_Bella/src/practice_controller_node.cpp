#include "SparkMax.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0;  // rpm, after gearbox turns into 11.1 RPM

enum CAN_IDs  // assign ID to each motor
{
  LEFT_MOTOR = 1,
  RIGHT_MOTOR = 2,
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
  private:
    SparkMax leftMotor;   // controls the left motor
    SparkMax rightMotor;  // controls the right motor

    // joy_subscriber_ is of type rclcpp::Subscription<general_msgs::msg::Joy>::SharedPtr
    // <general_msgs::msg::Joy> is the kind of message expected (messages of type Joy)
    // this is basically a shared pointer to a subscription that receives Joy messages
    rclcpp::Subscription<general_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    rclcpp::Client<controller_pkg::srv::ExcavationRequest>::SharedPtr excavation_client_;  // 
  public:

    // constructor
    ControllerNode(const std::string &can_interface) : Node("controller_node")  // initialize rclcpp::Node (the base ROS node) with the name "controller_node"
    {
      // Initialize left and right motors
      leftMotor = SparkMax(can_interface, LEFT_MOTOR);
      rightMotor = SparkMax(can_interface, RIGHT_MOTOR);

      // log start of new node creation
      RCLCPP_INFO(this->get_logger(), "Beginning node...");
      RCLCPP_INFO(this->get_logger(), "Configuring motor controllers...");

      // set the motor types
      leftMotor.SetMotorType(MotorType::kBrushless);
      rightMotor.SetMotorType(MotorType::kBrushless);
      // make the motors stay still
      leftMotor.SetIdleMode(IdleMode::kBrake);
      rightMotor.SetIdleMode(IdleMode::kBrake);
      // configure other stuff

      // listen to joystick input by subscribing to joystick
      joy_subscriber_ = this->create_subscription<general_msgs::msg::Joy>(       // joy_subscriber_ points to live ROS 2 subscription object 
        "/joy", 10,                                                              // 10 is the queue size. Node can hold up to 10 messags
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));  // call joy_callback function
      RCLCPP_INFO(this->get_logger(), "Joystick subscriber initialized.");

      // send those commands to motor controllers
      excavation_client_ = this->create_client<controller_pkg::srv::ExcavationRequest>("excavation_service");
      RCLCPP_INFO(this->get_logger(), "ControllerNode fully initialized.");
    }

};
