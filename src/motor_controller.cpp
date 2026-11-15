#include <iostream>
#include "motor_controller.hpp"

int main() {
    std::cout << "--- Initializing 4WD Rover Drivetrain ---\n";

    // 1. Instantiate 4 independent motors.
    // Parameters: Target Speed, Current Speed, kp, ki, kd
    
    // LEFT SIDE: Pushing hard at 50 RPM
    rover::MotorPIDController front_left(50.0, 0.0, 0.5, 0.1, 0.05);
    rover::MotorPIDController rear_left(50.0, 0.0, 0.5, 0.1, 0.05);

    // RIGHT SIDE: Braking at 20 RPM to force a sharp right turn
    rover::MotorPIDController front_right(20.0, 0.0, 0.5, 0.1, 0.05);
    rover::MotorPIDController rear_right(20.0, 0.0, 0.5, 0.1, 0.05);

    // 2. Execute the PID loops for the turn maneuver
    std::cout << "\n[Executing Left Side Acceleration...]\n";
    front_left.run_until_stabilized();
    rear_left.run_until_stabilized();

    std::cout << "\n[Executing Right Side Braking...]\n";
    front_right.run_until_stabilized();
    rear_right.run_until_stabilized();

    std::cout << "\n--- Turn Maneuver Complete ---\n";
    return 0;
}