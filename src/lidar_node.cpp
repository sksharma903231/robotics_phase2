#include <iostream>
#include <vector>
#include "lidar_processor.hpp"

int main()
{
    std::cout << "--C++ Lidar Node Initialized--\n";

    // Simulated LiDAR range scan data (meters) with noise, out-of-range, and valid readings
    std::vector<double> sensor_data = {12.5, 15.0, -1.0, 2.2, 20.0, 0.05, 5.0};

    rover::LidarProcessor processor(3.0, 6.0, 0.1, 50.0);
    rover::ObstacleStatus status = processor.processScan(sensor_data);

    if (status != rover::ObstacleStatus::NO_DATA) {
        std::cout << "Closest valid object is at: " << processor.getClosestDistance() << " meters\n";
    }

    std::cout << processor.getStatusString() << "\n";

    return (status == rover::ObstacleStatus::EMERGENCY_STOP) ? 0 : 0;
}