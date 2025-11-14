  }

    // Subscriber to joystick
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Joystick subscriber initialized");
  }

private:
  // Motors
  SparkMax leftMotor;
  SparkMax rightMotor;
  // Example for lifts and tilt
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;

  // ROS 2 subscriptions and clients
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Client<my_robot_pkg::srv::DigCommand>::SharedPtr excavation_client_;

  // Controller joystick callback
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    float leftJS = -joy_msg->axes[Gp::Axes::_LEFT_VERTICAL_STICK];
    float rightJS = -joy_msg->axes[Gp::Axes::_RIGHT_VERTICAL_STICK];

    float left_drive_raw = std::clamp(leftJS, -1.0f, 1.0f);
    float right_drive_raw = std::clamp(rightJS, -1.0f, 1.0f);

    float left_drive = computeStepOutput(left_drive_raw);
    float right_drive = computeStepOutput(right_drive_raw);

    leftMotor.SetDutyCycle(left_drive);
    rightMotor.SetDutyCycle(right_drive);

    // Heartbeat call to motors
    leftMotor.HeartBeat();
    rightMotor.HeartBeat();
    leftLift.HeartBeat();
    rightLift.HeartBeat();
  }

  float computeStepOutput(float value)
  {
    // Example transformation (can be replaced with proper scaling)
    return value;
  }

  void send_dig_command()
  {
    if (!excavation_client_)
    {
      RCLCPP_WARN(this->get_logger(), "Can't find excavation client");
      return;
    }

    auto dig_request = std::make_shared<my_robot_pkg::srv::DigCommand::Request>();
    dig_request->activate_digging = true;

    RCLCPP_INFO(this->get_logger(), "Sending excavation request");

    auto future_result = excavation_client_->async_send_request(
        dig_request,
        [this](rclcpp::Client<my_robot_pkg::srv::DigCommand>::SharedFuture response)
        {
          if (response.get()->accepted)
          {
            RCLCPP_INFO(this->get_logger(), "Digging successful");
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Digging rejected by server");
          }
        });
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
