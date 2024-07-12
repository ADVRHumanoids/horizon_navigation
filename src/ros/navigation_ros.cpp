#include <ros/navigation_ros.h>

NavigationROS::NavigationROS(double rate):
    _rate(rate)
{

    _nh = ros::NodeHandle("");
    _nhpr = ros::NodeHandle("~");

    _sonar_ros = std::make_unique<SonarOccupancyMapROS>();
    _velodyne_ros = std::make_unique<VelodyneOccupancyMapROS>(rate);

    _navigation = std::make_unique<Navigation>(_velodyne_ros->getVelodyneOccupancyMap(), _sonar_ros->getSonarOccupancyMap());

    _local_map_publisher = _nh.advertise<nav_msgs::OccupancyGrid>("navigation_map", 1, true);
    _local_sonar_map_publisher = _nh.advertise<nav_msgs::OccupancyGrid>("sonar_map", 1, true);

}

bool NavigationROS::update()
{
    
    auto _latest_local_map = _velodyne_ros->getSensedOccupancyMap();
    
    if (_latest_local_map)
    {

        auto base_link_T_map = _velodyne_ros->getTransformation(_latest_local_map->header.stamp);
        auto range_messages  = _sonar_ros->getRanges();
        _navigation->update(*_latest_local_map, base_link_T_map, range_messages);

        _local_grid_map = _navigation->getVelodyneLocalMap();
        _local_sonar_grid_map = _navigation->getSonarLocalMap();

        return true;
    }

    return false;
}

void NavigationROS::spin()
{
    while (_nh.ok())
    {
        try
        {
            auto success = update();

            if (success)
            {
            // publish grid_maps to ros occupancy maps
                nav_msgs::OccupancyGrid local_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_local_grid_map, "obstacles", -100, 100, local_occupancy_grid_map);
                _local_map_publisher.publish(local_occupancy_grid_map);

                nav_msgs::OccupancyGrid sonar_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_local_sonar_grid_map, "sonar", -100, 100, sonar_occupancy_grid_map);
                _local_sonar_map_publisher.publish(sonar_occupancy_grid_map);

//                nav_msgs::OccupancyGrid exclusion_zone_occupancy_grid_map;
//                grid_map::GridMapRosConverter::toOccupancyGrid(_exclusion_submap, _grid_map_layer_name, -100, 100, exclusion_zone_occupancy_grid_map);
//                _blind_zone_pub.publish(exclusion_zone_occupancy_grid_map);
            }

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        _rate.sleep();
        ros::spinOnce();
    }
}
