#ifndef SONAR_H
#define SONAR_H

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

class SonarOccupancyMap
{

public:

    typedef std::shared_ptr<SonarOccupancyMap> Ptr;

    struct Sonar
    {
        Eigen::Isometry3d origin_T_sensor;
        double detection_range;
        double arc_resolution;
    };

    SonarOccupancyMap(Eigen::Vector2d map_origin);

    bool addSensor(std::string name, Sonar sonar);

    bool setData(std::string name,
                 std::string frame_id,
                 double range,
                 double min_range,
                 double max_range,
                 double field_of_view);

    bool update();

    grid_map::GridMap getMap();




private:

    struct SonarUpdate
    {
        std::string frame_id;
        double range;
        double min_range;
        double max_range;
        double field_of_view;
//        double arc_resolution;
//        double detection_range;

    };

    void updateSonar(std::string name,
                     SonarUpdate sonar);

    grid_map::GridMap _map;

    std::map<std::string, Sonar> _sensors;
    std::map<std::string, SonarUpdate> _sensor_updates;

    grid_map::Position _map_origin, _sensor_origin;



};

#endif
