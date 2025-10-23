

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
  
    

}
