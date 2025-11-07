#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

ELOCITY_MAX = 2500.0;  // rpm, after gearbox turns into 11.1 RPM
const float VIBRATOR_OUTPUT = 1.0f; // Constant value for vibrator output

enum CAN_IDs
{
  LEFT_MOTOR = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT = 3,
  RIGHT_LIFT = 4,
  TILT = 5,
  VIBRATOR = 6
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
public:
ControllerNode(const std::string &can_interface)
    : Node("controller node"),
        leftMotor(can_interface, LEFT_MOTOR),
        rightMotor(can_interface, RIGHT_MOTOR),
        leftLift(can_interface, LEFT_LIFT),
        rightLift(can_interface, RIGHT_LIFT),
        tilt(can_interface, TILT),
        vibrator(can_interface, VIBRATOR),
        vibrator_active(false),
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

private:
  // Direct object members.
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedPtr depositing_client_;
  rclcpp::Client<interfaces_pkg::srv::ExcavationRequest>::SharedPtr excavation_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPub;
  rclcpp::TimerBase::SharedPtr timer;

  // Autonomy flag
  bool is_autonomy_active_ = false;

  // Vibrator toggle
  bool vibrator_active_;
  bool prev_vibrator_button_;

  // Alternate control mode toggle variables.
  bool alternate_mode_active_ = false;
  bool prev_alternate_button_ = false;

  float left_lift_position = 0.0f;
  float right_lift_position = 0.0f;

  // Helper for stepped output, in velocity control mode it is multiplied by VELOCITY_MAX
  /**
   * @brief Output must be bound within the range [-1.0,1.0].
   *        Returned value will be an integer multiple of 0.25.
   * @param value
   * @returns Step output of value.
   */
  float computeStepOutput(float value)
  {
    float absVal = std::fabs(value);
    if (absVal < 0.25f)
      return 0.0f;

    else if (absVal < 0.5f)
      return (value > 0 ? 0.25f : -0.25f);

    else if (absVal < 0.75f)
      return (value > 0 ? 0.5f : -0.5f);

    else if (absVal < 1.0f)
      return (value > 0 ? 0.75f : -0.75f);

    return (value > 0 ? 1.0f : -1.0f);
  }

  /**
   * @brief Sends request to depositing node and manages response
   * @param None
   * @returns None
   */
  void send_deposit_request()
  {
    if (!depositing_client_ || !depositing_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }
    auto request = std::make_shared<interfaces_pkg::srv::DepositingRequest::Request>();
    request->start_depositing = true;
    depositing_client_->async_send_request(request, [this](rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedFuture future)
                                           {
      auto response = future.get();
      if (response->depositing_successful) {
        RCLCPP_INFO(this->get_logger(), "Depositing successful");
      } else {
        RCLCPP_WARN(this->get_logger(), "Depositing failed");
      } });
  })

//trying to work on new logic for excavation 
void joyCallback(const sensor_msg::msg::Joy::SharedPtr joy){
    bool vibrator_button = (joy->buttons[5] > 0);
    if(vibrator_button && !prev_vibrator_button) //if you are currently pressing the button, then set the activity to active
    {
        vibrator_active = true;
        RCLCPP_(get_logger(), "Vibrator toggled %s", vibrator_active_ ? "ON" : "OFF");
    }
    prev_vibrator_button = vibrator_button;
    
    float vibrator_duty = vibrator_active_ ? VIBRATOR_OUTPUT : 0.0f;
    setVibratorDuty(vibrator_duty);

    //Excaation reset (x button)
    if(joy->buttons[2] > 0) //x button
    {
        resetExcavation();
        return -1;
    }

    //Tilt actuator (D-pad left/right)
    float tilt_duty = 0.0f;
    if(joy->axes[6] > 0.5) tilt_duty = 1.0f; //left
    else if (joy->axes[6] < -0.5) tilt_duty = 1.0f; //right
    tilt.setTiltDuty(tilt_duty);

    //lift actuator (d-pad up/down)
    float lift_duty = 0.0f
    if(joy->axes[7] > 0.5) lift_duty = 1.0f; //up
    else if (joy->axes[7] < -0.5) lift_duty = -1.0f; //down

    int main(int argc, chart *argv[])
    {
        
        rclcpp::inint(argc, argv);
        std::string can_interface = "can0";
        auto temp_node = rclcpp::Node::make_shared("fart_node");
        rclcpp::spin(std::make_shared<ExcavationControlNode>());
        rclcpp::shutdown();
        return 0;
    }
