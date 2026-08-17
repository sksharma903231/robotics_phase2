#include<iostream>
#include<vector>

int main()
{
    std::cout<<"--C++ Lidar Node Initialized--\n";
    std::vector<int> sensor_data={12,15,2,20,5};
    int closest_object=999;

    for (int distance: sensor_data)
    {
        if (distance < closest_object)
        {
            closest_object=distance;
        }
    }

    if (closest_object < 3) 
    {
        std::cout<<"CRITICAL: Obstacle too close, breakes applied !\n";
    }
    else {
        std::cout<<"Path is Clear.";
    }

    return 0;

}