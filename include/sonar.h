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

    SonarOccupancyMap(Eigen::Vector2d map_origin,
                      std::map<std::string, Eigen::Isometry3d> sonar_map);

    bool setData(std::string frame_id,
                 double range,
                 double min_range,
                 double max_range,
                 double field_of_view);

    bool update();

    grid_map::GridMap getMap();




private:

    struct SonarInfo
    {
        std::string frame_id;
        double range;
        double min_range;
        double max_range;
        double field_of_view;
        double arc_resolution;
        double detection_range;
        Eigen::Isometry3d origin_T_sensor;
    };

    void updateSonar(SonarInfo sonar);

    grid_map::GridMap _map;

    std::map<std::string, SonarInfo> _sonar_info;

    grid_map::Position _map_origin, _sensor_origin;



};

#endif
