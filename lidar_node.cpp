#include <iostream>
#include <vector>

int main()
{
    std::cout << "--C++ Lidar Node Initialized--\n";
    
    std::vector<double> sensor_data = {12.5, 15.0, -1.0, 2.2, 20.0, 0.05, 5.0};
    double closest_object = 999.0;

    
    for (double distance : sensor_data)
    {
        
        if (distance < closest_object && distance > 0.1)
        {
            closest_object = distance;
        }
    }
    
    std::cout << "Closest valid object is at: " << closest_object << " meters\n";
    
    if (closest_object < 3.0) 
    {
        std::cout << "CRITICAL: Obstacle too close, brakes applied!\n";
    }
    else 
    {
        std::cout << "Path is Clear.\n";
    }

    return 0;
}