#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0;

enum CAN_IDs
{
  LEFT_MOTOR = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT = 3,
  RIGHT_LIFT = 4,
};

namespace Gp
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


class ControllerNode : public rclcpp::Node
{

  private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMax leftLift;
    SparkMax rightLift;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joysubscriber;
  
  public:
    ControllerNode(const std::string &can_interface)
        : Node("controller_node"),
          leftMotor(can_interface, LEFT_MOTOR),
          rightMotor(can_interface, RIGHT_MOTOR),
          leftLift(can_interface, LEFT_LIFT),
          rightLift(can_interface, RIGHT_LIFT),
          tilt(can_interface, TILT),
          vibrator(can_interface, VIBRATOR),
          vibrator_active_(false),
          prev_vibrator_button_(false),
          alternate_mode_active_(false),
          prev_alternate_button_(false)
    {
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
  
      tilt.SetIdleMode(IdleMode::kBrake);
      tilt.SetMotorType(MotorType::kBrushed);
      tilt.SetSensorType(SensorType::kEncoder);
      // Initializes the settings for the tilt actuator
  
      vibrator.SetIdleMode(IdleMode::kBrake);
      vibrator.SetMotorType(MotorType::kBrushed);
      vibrator.SetSensorType(SensorType::kEncoder);
      // Initializes the settings fro the vibrator
  
      leftMotor.SetInverted(false);
      rightMotor.SetInverted(true);
      leftLift.SetInverted(true);
      rightLift.SetInverted(true);
      tilt.SetInverted(true);
      vibrator.SetInverted(true);
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
  
      // PID settings for tilt
      tilt.SetP(0, 1.51f);
      tilt.SetI(0, 0.0f);
      tilt.SetD(0, 0.0f);
      tilt.SetF(0, 0.00021f);
      // PID settings for tilt
  
      leftMotor.BurnFlash();
      rightMotor.BurnFlash();
      leftLift.BurnFlash();
      rightLift.BurnFlash();
      tilt.BurnFlash();
      vibrator.BurnFlash();
      RCLCPP_INFO(this->get_logger(), "Motor Controllers Initialized");
  
      // ---ROS SUBSCRIPTIONS--- //
      RCLCPP_INFO(this->get_logger(), "Initializing Joy Subscription");
      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy", 10,
          std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Joy Subscription Initialized");
  
      health_subscriber_ = this->create_subscription<interfaces_pkg::msg::MotorHealth>(
          "/health_topic", 10,
          std::bind(&ControllerNode::position_callback, this, std::placeholders::_1));
  
      RCLCPP_INFO(this->get_logger(), "Initializing depositing, excavation, and travel client");
      depositing_client_ = (this->create_client<interfaces_pkg::srv::DepositingRequest>("depositing_service"));
      excavation_client_ = (this->create_client<interfaces_pkg::srv::ExcavationRequest>("excavation_service"));
      RCLCPP_INFO(this->get_logger(), "Excavation, depositing clients initialized");
  
      RCLCPP_INFO(this->get_logger(), "Initializing Heartbeat Publisher");
      heartbeatPub = this->create_publisher<std_msgs::msg::String>("/heartbeat", 10);
      RCLCPP_INFO(this->get_logger(), "Heartbeat Publisher Initialized");
  
      RCLCPP_INFO(this->get_logger(), "Initializing Timer");
      timer = this->create_wall_timer(
          std::chrono::milliseconds(1000),
          std::bind(&ControllerNode::publish_heartbeat, this));
      RCLCPP_INFO(this->get_logger(), "Timer Initialized");
  
      RCLCPP_INFO(this->get_logger(), "Node Initialization Complete");
    }
    //drive train
    void handle_drive_train(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
      float left_drive = 0.0f;
      float right_drive = 0.0f;
      float left_drive_raw = 0.0f;
      float right_drive_raw = 0.0f;
      float left_lift = 0.0f;
      float right_lift = 0.0f;
      float lift_raw = 0.0f;

      float leftJS = -(joy_msg.axes(Gp::Axes::_LEFT_VERTICAL_STICK));
      float rightJS = -(joy_msg.axes(Gp::Axes::_RIGHT_VERTICAL_STICK));

      // keeps number between -1.0 and 1.0
      left_drive_raw = std::max(-1.0f, std::min(1.0f, leftJS));
      right_drive_raw = std::max(-1.0f, std::min(1.0f, rightJS));

      float lift_trig = (joy_msg.axes(Gp::Axes::_LEFT_TRIGGER));
      // lift stuff
      lift_raw = std::max(0.0f, std::min(1.0f, lift_trig));
      
      // how to get position?? ask later about this. is it in health node and do I need that first??
      left_drive = computeStepOutput(left_drive_raw);
      right_drive = computeStepOutput(right_drive_raw);

      leftMotor.SetDutyCycle(left_drive);
      rightMotor.SetDutyCycle(right_drive);
      leftLift.SetPosition(lift_raw);
      rightLift.SetPosition(lift_raw);
      
      leftMotor.HeartBeat();
      rightMotor.HeartBeat();
      leftLift.Heartbeat();
      rightLift.Heartbeat();
    }  

    // void publish_heartbeat()
    // {
    //   auto msg = std_msgs::msg::String();
    //   msg.data = "Heartbeat";
    //   heartbeatPub->publish(msg);
    // }

}



int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  std::string can_interface = "can0";
  auto temp_node = std::make_shared<ControllerNode>(can_interface);
  RCLCP_INFO(this->get_logger(), "started controller node");
  rclcpp::spin(temp_node);
  rclcpp::shutdown();
  return 0;
}
