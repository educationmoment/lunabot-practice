#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"

const float VIBRATOR_DUTY = 1.0f;
const float ERROR = 0.1f;
float buffer = 0.0f;

SparkMax leftDrive("can0", 1);
SparkMax rightDrive("can0", 2);
SparkMax leftLift("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax tilt("can0", 5);
SparkMax vibrator("can0", 6); //Initalizes motor controllers

rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;
std::shared_ptr<rclcpp::Node> node;

void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, float drive_speed)
{

}

void updateTiltPosition(const interfaces_pkg::msg::MotorHealth::SharedPtr health_msg){
    buffer = health_msg->tilt_position;
}

void Excavate(const std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request> request, std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Response> response)
{
    
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 

    node = rclcpp::Node::make_shared("excavation_node");

    rclcpp::Service<interfaces_pkg::srv::ExcavationRequest>::SharedPtr service =
    node->create_service<interfaces_pkg::srv::ExcavationRequest>("excavation_service", &Excavate);

    health_subscriber_ = node->create_subscription<interfaces_pkg::msg::MotorHealth>(
    "/health_topic", 10, updateTiltPosition);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Initalized");

    rclcpp::spin(node);
    rclcpp::shutdown();
}