#include <iostream>
#include<cmath>

class MotorController {
private:
    // 1. DATA ZONE: Declare all your variables here.
    double target_speed;
    double current_speed;
    double kp, ki, kd;
    double error, previous_error, integral_sum;

public:
    // 2. CONSTRUCTOR: This setup function runs automatically when a motor is created.
    // It assigns the starting values to our private variables.
    MotorController(double target, double current, double p, double i, double d) {
        target_speed = target;
        current_speed = current;
        kp = p;
        ki = i;
        kd = d;
        
        // Initialize the tracking variables
        error = target_speed - current_speed;
        previous_error = error;
        integral_sum = 0.0;
    }

    // 3. ACTION ZONE: This is where the hardware does the work.
    void run_pid_loop() {
        std::cout << "--- Initializing Motor Node ---\n";
        
        while(std::abs(error) > 0.1)
        {
            // 1. Proportional Calculation (The Present)
            double p = kp * error;
            // 2. Integral Calculation (The Past)
            integral_sum = integral_sum + error;
            double i = ki * integral_sum;
            // 3. Derivative Calculation (The Future)
            double d_correction = kd * (error - previous_error);
            // 4. Apply the combined voltage to the motor
            current_speed = current_speed + p + i + d_correction;
            // 5. Update the tracking variables for the next loop
            previous_error = error; 
            error = target_speed - current_speed; 
            
            std::cout << "Speed: " << current_speed << " RPM | Error: " << error << "\n";
        }
        std::cout << "Final Stabilized Speed: " << current_speed << " RPM\n";
    }
};


int main() {
    std::cout << "--- Initializing 4WD Rover Drivetrain ---\n";

    // 1. Instantiate 4 independent motors.
    // Parameters: Target Speed, Current Speed, kp, ki, kd
    
    // LEFT SIDE: Pushing hard at 50 RPM
    MotorController front_left(50.0, 0.0, 0.5, 0.1, 0.05);
    MotorController rear_left(50.0, 0.0, 0.5, 0.1, 0.05);

    // RIGHT SIDE: Braking at 20 RPM to force a sharp right turn
    MotorController front_right(20.0, 0.0, 0.5, 0.1, 0.05);
    MotorController rear_right(20.0, 0.0, 0.5, 0.1, 0.05);

    // 2. Execute the PID loops for the turn maneuver
    std::cout << "\n[Executing Left Side Acceleration...]\n";
    front_left.run_pid_loop();
    rear_left.run_pid_loop(); // Uncommented to run

    std::cout << "\n[Executing Right Side Braking...]\n";
    front_right.run_pid_loop();
    rear_right.run_pid_loop(); // Uncommented to run

    std::cout << "\n--- Turn Maneuver Complete ---\n";
    return 0;
}