#include <sonar.h>

SonarOccupancyMap::SonarOccupancyMap(Eigen::Vector2d map_origin): //= std::vector<Eigen::Isometry3d>
    _map({"sonar_map"}),
    _map_origin(map_origin)
//    _detection_range(detection_range)
{


    _map.setPosition(_map_origin);
    _map.setFrameId("base_link");
    _map.setGeometry(grid_map::Length(4.0, 4.0), 0.01);

    ROS_INFO("Created map with size %f x %f m (%i x %i cells).", _map.getLength().x(),
                                                                 _map.getLength().y(),
                                                                 _map.getSize()(0),
                                                                 _map.getSize()(1));

//    double default_detection_range = 0.4;
//    double default_arc_resolution = 20;

//    for (auto sonar : sonar_transform)
//    {
//        SonarSensor sonar_info;
//        sonar_info.frame_id = sonar.first;
//        sonar_info.origin_T_sensor = sonar.second;
//        sonar_info.arc_resolution = default_arc_resolution;

//        if (sonar_detection_range.find(sonar.first) == sonar_detection_range.end())
//        {
//          sonar_info.detection_range = default_detection_range;
//        }
//        else
//        {
//          sonar_info.detection_range = sonar_detection_range[sonar.first];
//        }
//        _sensors[sonar.first] = sonar_info;
//    }

}

bool SonarOccupancyMap::addSensor(std::string name, SonarOccupancyMap::Sonar sonar)
{
    _sensors[name].detection_range = sonar.detection_range;
    _sensors[name].arc_resolution = sonar.arc_resolution;
    _sensors[name].origin_T_sensor = sonar.origin_T_sensor;

    std::cout << "adding sensor " << "'" << name << "'" << std::endl;
    std::cout << "       detection range: " << _sensors[name].detection_range << std::endl;
    std::cout << "       arc_resolution: " << _sensors[name].arc_resolution << std::endl;
    std::cout << "       transform: " << _sensors[name].origin_T_sensor.translation().transpose() << std::endl;
    std::cout << std::endl;

    return true;
}

bool SonarOccupancyMap::setData(std::string name, std::string frame_id, double range, double min_range, double max_range, double field_of_view)
{
    _sensor_updates[name].frame_id = frame_id;
    _sensor_updates[name].range = range;
    _sensor_updates[name].min_range = min_range;
    _sensor_updates[name].max_range = max_range;
    _sensor_updates[name].field_of_view = field_of_view;

    return true;
}


bool SonarOccupancyMap::update()
{

    _map.clear("sonar_map");

    for (auto sonar_update : _sensor_updates)
    {
        updateSonar(sonar_update.first, sonar_update.second);
    }

    return true;
}

grid_map::GridMap SonarOccupancyMap::getMap()
{
    return _map;
}

void SonarOccupancyMap::updateSonar(std::string sensor_name,
                                    SonarOccupancyMap::SonarUpdate sonar_update)
{
    auto sonar = _sensors[sensor_name];

    if (sonar_update.range <= sonar.detection_range)
    {
        for (auto iterator = 0; iterator < sonar.arc_resolution; iterator++)
        {
            // compute the range
            double angle = - sonar_update.field_of_view / 2 + iterator * (sonar_update.field_of_view / sonar.arc_resolution);
            double x_cell = sonar_update.range * cos(angle);
            double y_cell = sonar_update.range * sin(angle);


            grid_map::Position3 point_sensor3d(x_cell, y_cell, 0.0);
            auto point_world3d = sonar.origin_T_sensor * point_sensor3d;

            grid_map::Position position(point_world3d.x(), point_world3d.y());

            _map.atPosition("sonar_map", position) = 100;
        }
    }
}
