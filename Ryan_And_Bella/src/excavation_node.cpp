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


void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, float drive_speed) {
    auto timer_start = std::chrono::high_resolution_clock::now();
    bool leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
    bool rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
    bool tiltReached = (fabs(tilt_setpoint - tilt.GetPosition()) <=  ERROR);

    while (!((leftLiftReached && rightLiftReached) && tiltReached)){
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.2){
            if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.75){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WARNING: ACTUATORS GREATELY MISALIGNED");
            }
            leftLift.SetPosition(rightLift.GetPosition());
            rightLift.SetPosition(rightLift.GetPosition());
        } //block for lift realignment
        else {
            leftLift.SetPosition(lift_setpoint);
            rightLift.SetPosition(lift_setpoint);
            tilt.SetPosition(tilt_setpoint);
        } //block for normal bucket movement

        if (activate_vibrator) vibrator.SetDutyCycle(VIBRATOR_DUTY);
        
        leftDrive.SetVelocity(drive_speed);
        rightDrive.SetVelocity(drive_speed);

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - timer_start).count() > 5) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Skipping stage...");
            break;
        } //Timer for when to quit a stage due to timeout
        
        leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
        rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
        tiltReached = (fabs(tilt_setpoint - tilt.GetPosition() ) <=  ERROR); //Updates statuses
    }
}

void Excavate(const std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Response> response) {

        MoveBucket(-3.0,-3.0 + buffer, true, 1500);
    
        auto dig_timer1 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - digtimer1).count() < 2){
            leftdrive.SetVelocity(800.0f);
            rightdrive.SetVelocity(800.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-3.9, -3.8 + buffer, true, 800);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        auto dig_timer2 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - digtimer2).count() < 2){
            leftdrive.SetVelocity(1500.0f);
            rightdrive.SetVelocity(1500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-3.9, -3.3 + buffer, true, 1500);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        auto dig_timer3 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - digtimer3).count() < 2){
            leftdrive.SetVelocity(1000.0f);
            rightdrive.SetVelocity(1000.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-3.8, -3.0 + buffer, true, 1000);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        auto dig_timer4 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - digtimer4).count() < 2){
            leftdrive.SetVelocity(500.0f);
            rightdrive.SetVelocity(500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-3.8, -3.0 + buffer, true, 500);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        MoveBucket(0.0, 0.0 + buffer, false, 0.0f); //Resets bucket

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
