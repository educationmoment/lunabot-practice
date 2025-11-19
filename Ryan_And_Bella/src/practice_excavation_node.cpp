#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"

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
