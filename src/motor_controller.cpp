#include <iostream>

int main()
{
    std::cout << "--- Full PID Motor Controller Initialized ---\n";

    double target_speed = 50.0;
    double current_speed = 40.0;
    
    // Tuning Constants
    double kp = 0.5;
    double ki = 0.1;
    double kd = 0.05; // The Derivative dampener
    
    double error = target_speed - current_speed;
    double p_correction = 0;
    double i_correction = 0;
    double integral_sum = 0; 
    
    // Remember the previous error to calculate the rate of change
    double previous_error = error; 

    while(error > 0.1)
    {
        // 1. Proportional Calculation (The Present)
        p_correction = kp * error;
        
        // 2. Integral Calculation (The Past)
        integral_sum = integral_sum + error;
        i_correction = ki * integral_sum;

        // 3. Derivative Calculation (The Future)
        double d_correction = kd * (error - previous_error);

        // 4. Apply the combined voltage to the motor
        current_speed = current_speed + p_correction + i_correction + d_correction;
        
        // 5. Update the tracking variables for the next loop
        previous_error = error; // Save this before recalculating the new error
        error = target_speed - current_speed; 
        
        std::cout << "Speed: " << current_speed << " RPM | Error: " << error << "\n";
    }

    std::cout << "Final Stabilized Speed: " << current_speed << " RPM\n";

    return 0; 
}