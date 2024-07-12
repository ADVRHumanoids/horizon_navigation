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
        typedef std::shared_ptr<Sonar> Ptr;
        Eigen::Isometry3d origin_T_sensor;
        double detection_range;
        double arc_resolution;
    };

    SonarOccupancyMap(Eigen::Vector2d map_origin);

    bool addSensor(std::string name, Sonar::Ptr sonar);

    Sonar::Ptr getSensor(std::string sonar_name);

    std::vector<std::string> getSensorsNames();



    bool setData(std::string name,
                 std::string frame_id,
                 double range,
                 double min_range,
                 double max_range,
                 double field_of_view);

    bool update();

    grid_map::GridMap getMap();

    bool clearSonarCone(grid_map::GridMap& grid_map,
                        std::string sensor_name,
                        std::string layer_name,
                        Eigen::Isometry3d map_transform = Eigen::Isometry3d::Identity());

    void updateGridMapWithSonar(grid_map::GridMap& grid_map,
                                std::string sensor_name,
                                std::string layer_name,
                                Eigen::Isometry3d map_transform = Eigen::Isometry3d::Identity());

    bool getSensorsStatus(std::string sensor_name);


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



    grid_map::GridMap _map;
    grid_map::Position _map_origin, _sensor_origin;

    std::map<std::string, Sonar::Ptr> _sensors;
    std::map<std::string, SonarUpdate> _sensor_updates;
    std::vector<std::string> _sensors_names;




};

#endif
