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

/*
 * 
 *  Some Todos:
 *  get parameters of luna parts
 *  to be able to predict motion in future 
 * 
 *  Drive speed of motors range ( - 1500)
 * 
 *  1. Are there time constraints for you to perform fucntions?
 * 
 *  2. Drive speed and distance covered at diff speeds?
     * measure distance travelled for each speed,
     * divide distance by time to get approximate m/s for each command?
     

 */

/**
 * @brief MoveBucket positions the bucket to a predefined position specified in the passed
 *        parameters. By choice, the vibrator can be enabled to assist in cleaving through regolith.
 *        Motor output during excavation cycle is also passed to function call.
 * @param lift_setpoint Desired lift actuator setpoint expressed as a float
 * @param tilt_setpoint Desired tilt actuator setpoint expressed as a float
 * @param activate_vibrator Should the vibrator be activated? (true == yes)
 * @param drive_speed Floating point value used to express desired motor speed during auto excavation
 * @returns None
 */
void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, float drive_speed) {
    auto timer_start = std::chrono::high_resolution_clock::now();
    bool leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
    bool rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
    bool tiltReached = (fabs(tilt_setpoint - tilt.GetPosition()) <=  ERROR);

    while (!((leftLiftReached && rightLiftReached) && tiltReached)){
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing

        // Keep the two lift actuators aligned to avoid twisting the structure
        if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.2f){
            if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.75f){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WARNING: ACTUATORS GREATLY MISALIGNED");
            }
            leftLift.SetPosition(rightLift.GetPosition());
            rightLift.SetPosition(rightLift.GetPosition());
        } 
        else {
            leftLift.SetPosition(lift_setpoint);
            rightLift.SetPosition(lift_setpoint);
            tilt.SetPosition(tilt_setpoint);
        }

        if (activate_vibrator) {
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
        }
        
        leftDrive.SetVelocity(drive_speed);
        rightDrive.SetVelocity(drive_speed);

        // Safety timeout so we don't sit forever if a sensor dies
        if (std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - timer_start).count() > 5) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Skipping stage...");
            break;
        }
        
        leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
        rightLiftReached = (fabs(tilt_setpoint - rightLift.GetPosition() ) <=  ERROR); 
        tiltReached = (fabs(tilt_setpoint - tilt.GetPosition()) <=  ERROR);
    }
}

/**
 * @param health_msg interfaces_pkg::msg::MotorHealth, tilt_position read from node and stored in buffer
 * @returns None
 */
void updateTiltPosition(const interfaces_pkg::msg::MotorHealth::SharedPtr health_msg){
    buffer = health_msg->tilt_position;
}

/**
 * @brief Callback for interfaces_pkg::srv::ExcavationRequest::Request interface. Handles autonomous excavation.
 * @param request std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request>, client provided request
 * @param response std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request>, server response
 * @returns None
 */

void Excavate(const std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Response> response) {
        if (!request->start_excavation){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received request but start_excavation is false");
            response->excavation_successful = false;
            return;
        } //Checks to make sure start_excavation is set to true

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation request received, current tilt buffer = %f", buffer);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting excavation process");

 
        // Stage 1 – Approach pose
        // MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, float drive_speed)
        // We want the bucket just above the surface before we start cutting.
        // Assumption (from previous tuning):
        //     lift ≈ -2.5, 
        //     tilt ≈ -2.6 + buffer => bucket edge near the surface,
        //     with a small nose-down attitude.
        
        MoveBucket(-2.5f, -2.6f + buffer, false, 0.0f);
    
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 1 complete: bucket approach pose");

 
        // Stage 2 – Initial cut
 
        // We drop slightly deeper and, 
        //    begin cutting forward with the vibrator on.
        //    lift:  -3.0 is deeper than -2.5 
        //    tilt:  -3.0 + buffer keeps bucket slightly nose down to cut in.
        //    drive_speed: 1500.
        //
        MoveBucket(-3.0f, -3.0f + buffer, true, 1500.0f); 
        // MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, float drive_speed)

        auto dig_timer1 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - dig_timer1).count() < 2){
            leftDrive.SetVelocity(1500.0f);
            rightDrive.SetVelocity(1500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);

            // Slightly deeper + slight nose-up to help collect material:
            //    lift: -3.6 (deeper)
            //    tilt: -3.5 + buffer => (tilt - lift) ≈ +0.1 in raw units
            //     -> small nose up angle to keep sand in the bucket.
            MoveBucket(-3.6f, -3.5f + buffer, true, 1500.0f);

            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 2 complete: aggressive cut and initial fill");

 
        // Stage 3 – Scoop & partial lift
 
        // we are trying to maintain depth but rotate bucket further nose up so it holds
        // more material, and reduce drive speed to avoid spilling.

        // lift stays at -3.6 (roughly same as Stage 2).
        // tilt moves closer to buffer ( -2.9 + buffer), so
        // tilt - lift ≈ positive -> nose up.
        //  drive_speed reduced from 1500 to 1200 (slower travel). 
    
        auto dig_timer2 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - dig_timer2).count() < 2){
            leftDrive.SetVelocity(1200.0f);
            rightDrive.SetVelocity(1200.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);

            MoveBucket(-3.6f, -2.9f + buffer, true, 1200.0f);
            // Stage 2-3 -> no changes to lift, tilt closer to buffer and slower speed?

            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 3 complete: scoop & partial lift");

 
        // Stage 4 – Carry 
 
        // now we slightly raise the arm and keep the bucket nose-up while
        // slowing the drivetrain to "carry" material out of the trench.

        //
        //  lift: -3.2  -> slightly raised vs Stage 3.
        //  tilt: -2.3 + buffer -> stronger nose-up (tilt - lift ~ more positive).
        //  drive_speed: 800 -> slower to minimize spillage? 
        //
        auto carry_timer = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - carry_timer).count() < 3){
            leftDrive.SetVelocity(800.0f);
            rightDrive.SetVelocity(800.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);

            MoveBucket(-3.2f, -2.3f + buffer, true, 800.0f); 
            // Stage 3-4 lift raised, tilt raised,  speed slower

            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 4 complete: carry material while exiting trench");

 
        // Stage 5 – stop motio

        leftDrive.SetDutyCycle(0.0f);
        rightDrive.SetDutyCycle(0.0f);
        vibrator.SetDutyCycle(0.0f);

        // Reset the bucket to a neutral  pose.
        MoveBucket(0.0f, 0.0f + buffer, false, 0.0f);

        // Small extra tilt "bump" to take up backlash if needed.
        auto reset_tilt = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - reset_tilt).count() < 1){
            tilt.SetDutyCycle(1.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        }

        response->excavation_successful = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 5 complete: Excavation sequence completed successfully");

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
