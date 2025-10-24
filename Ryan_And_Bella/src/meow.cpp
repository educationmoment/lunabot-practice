

Class ControllerNode : public rclcpp::Node
{
  public:

  ControllerNode(const std::string &can_interface)
  : Node("controller_node"),
    leftMotor(can_interface, LEFT_MOTOR),
    rightMotor(can_interface, RIGHT_MOTOR)
    {
      RCLCPP_INFO(this.get_logger(), "beginning node")
      RCLCPP_INFO(this.get_logger(), "configuring  motor controllers")

      leftMotor.SetIdleMode(IdleMode::kBrake);
      rightMotor.SetIdleMode(IdleMode::kBrake);
      leftMotor.SetMotorType(MotorType::kBrushless);
      rightMotor.SetMotorType(MotorType::kBrushless);

      // finish configuring 
      // leftMotor.BurnFlash();
      // rightMotor.BurnFlash();

    //drive train
    {
      float left_drive = 0.0
      float right_drive = 0.0
      float left_drive_raw = 0.0
      float right_drive_raw = 0.0

      float leftJS = joy_msg.axes(Gp::Axes::_LEFT_VERTICAL_STICK);
      float rightJS = joy_msg.axes(Gp::Axes::_RIGHT_VERTICAL_STICK);

      left_drive_raw = std::max(-1.0f, std::min(1.0f, leftJS));
      right_drive_raw = std::max(-1.0f, std::min(1.0f, rightJS));

      left_drive = computeStepOutput(left_drive_raw);
      right_drive = computeStepOutput(right_drive_raw);

      leftMotor.SetDutyCycle(left_drive);
      rightMotor.SetDutyCycle(right_drive);
      leftMotor.HeartBeat();
      rightMotor.HeartBeat();
    }  

    void publish_heartbeat()
    {
      auto msg = std_msgs::msg::String();
      msg.data = "Heartbeat";
      heartbeatPub.publish(msg);
    }

}

int main(int argc, char **argv){
}
