#include <sonar.h>

SonarOccupancyMap::SonarOccupancyMap(Eigen::Vector2d map_origin,
                                     std::map<std::string, Eigen::Isometry3d> sonar_map): //= std::vector<Eigen::Isometry3d>
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


    double detection_range = 0.4;
    double arc_resolution = 20;

    for (auto sonar : sonar_map)
    {
        SonarInfo sonar_info;
        sonar_info.frame_id = sonar.first;
        sonar_info.origin_T_sensor = sonar.second;
        sonar_info.arc_resolution = arc_resolution;
        sonar_info.detection_range = detection_range;
        _sonar_info[sonar.first] = sonar_info;
    }

    for (auto sonar : _sonar_info)
    {
        std::cout << sonar.second.detection_range << std::endl;
    }

    std::cout << "cazzi" << std::endl;
//    grid_map::Position3 map_origin_3d(_map_origin.x(), _map_origin.y(), 0.0);


//    auto sensor_origin3d = _origin_T_sensor * map_origin_3d;

//    _sensor_origin << sensor_origin3d.x(), sensor_origin3d.y();

////    _sensor_yaw = atan2(_origin_T_sensor.rotation()(2, 0), _origin_T_sensor.rotation()(0, 0));
}

bool SonarOccupancyMap::setData(std::string frame_id, double range, double min_range, double max_range, double field_of_view)
{
    _sonar_info[frame_id].range = range;
    _sonar_info[frame_id].min_range = min_range;
    _sonar_info[frame_id].max_range = max_range;
    _sonar_info[frame_id].field_of_view = field_of_view;

    return true;
}


bool SonarOccupancyMap::update()
{

    _map.clear("sonar_map");

    for (auto sonar : _sonar_info)
    {
        updateSonar(sonar.second);
    }


    return true;
}

grid_map::GridMap SonarOccupancyMap::getMap()
{
    return _map;
}

void SonarOccupancyMap::updateSonar(SonarOccupancyMap::SonarInfo sonar)
{

    std::cout << sonar.range << std::endl;
    std::cout << sonar.detection_range << std::endl;
    std::cout << sonar.arc_resolution << std::endl;


    if (sonar.range <= sonar.detection_range)
    {

        for (auto iterator = 0; iterator < sonar.arc_resolution; iterator++)
        {
            // compute the range
            double angle = - sonar.field_of_view / 2 + iterator * (sonar.field_of_view / sonar.arc_resolution);
            double x_cell = sonar.range * cos(angle);
            double y_cell = sonar.range * sin(angle);


            grid_map::Position3 point_sensor3d(x_cell, y_cell, 0.0);
            auto point_world3d = sonar.origin_T_sensor * point_sensor3d;

            grid_map::Position position(point_world3d.x(), point_world3d.y());

            _map.atPosition("sonar_map", position) = 100;
        }
    }
}
