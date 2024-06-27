#ifndef OBSTACLE_GENERATOR_ROS_H
#define OBSTACLE_GENERATOR_ROS_H

#include <obstacles_generator.h>

class ObstacleGeneratorROS
{
    public:
        ObstacleGeneratorROS(std::string name, int grid_width, int grid_height, int grid_resolution, std::list<std::string> args);
        void run();

    private:

};

#endif // OBSTACLE_GENERATOR_ROS_H
