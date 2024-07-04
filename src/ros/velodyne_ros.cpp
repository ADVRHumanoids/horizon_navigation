#include <ros/velodyne_ros.h>

VelodyneOccupancyMapROS::VelodyneOccupancyMapROS(double rate):
    _tfBuffer(),
    _tfListener(_tfBuffer),
    _rate(rate)
{

    _nh = ros::NodeHandle("");
    _nhpr = ros::NodeHandle("~");

    // get param from param server
    load_params();

    // subscribe to localmap
    init_subscribers();

    // open occupancy map publishers
    init_publishers();

    grid_map::Position map_origin(0.0, 0.0);
    double map_resolution = 0.01;

    _velodyne_occupancy_map = std::make_unique<VelodyneOccupancyMap>(map_origin,
                                                         grid_map::Length(_map_width, _map_height),
                                                         grid_map::Length(_blind_zone_width, _blind_zone_height),
                                                         map_resolution,
                                                         _grid_map_layer_name);

    // initialize sonar ros server
//    _sonar_ros = std::make_unique<SonarROS>();

}

bool VelodyneOccupancyMapROS::load_params()
{

    _nhpr.param<double>("map_width", _map_width, 6);
    ROS_INFO("Map width: %f", _map_width);

    _nhpr.param<double>("map_height", _map_height, 6);
    ROS_INFO("Map height: %f", _map_height);

    _nhpr.param<double>("blind_zone_width", _blind_zone_width, 1);
    ROS_INFO("Blind zone width: %f", _blind_zone_width);

    _nhpr.param<double>("blind_zone_height", _blind_zone_height, 1);
    ROS_INFO("Blind zone height: %f", _blind_zone_height);

    _nhpr.param<std::string>("grid_map_layer_name", _grid_map_layer_name, "obstacle_layer"); // Get string parameter from the server
    ROS_INFO("Grid map layer name: %s", _grid_map_layer_name.c_str());

    return true;
}

void VelodyneOccupancyMapROS::init_subscribers()
{
    _local_map_sub = _nh.subscribe("/local_map", 1, &VelodyneOccupancyMapROS::localMapCallback, this);
//    auto msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/local_map", _nh);

}

void VelodyneOccupancyMapROS::init_publishers()
{
    _navigation_map_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/velodyne_map", 1);
    _blind_zone_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/blind_zone", 1);
}


void VelodyneOccupancyMapROS::localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    _latest_local_map = map_msg;
}

bool VelodyneOccupancyMapROS::update()
{
    if (_latest_local_map)
    {
        auto base_link_T_map = getTransformation(_latest_local_map->header.stamp);
        _velodyne_occupancy_map->update(*_latest_local_map, base_link_T_map);

        _local_grid_map = _velodyne_occupancy_map->getLocalMap();
        _exclusion_submap = _velodyne_occupancy_map->getBlindZone();

        return true;
    }

    return false;
}

grid_map::GridMap VelodyneOccupancyMapROS::getMap()
{
    return _local_grid_map;
}

grid_map::GridMap VelodyneOccupancyMapROS::getBlindZone()
{
    return _exclusion_submap;
}

VelodyneOccupancyMap::Ptr VelodyneOccupancyMapROS::getVelodyneOccupancyMap()
{
    return _velodyne_occupancy_map;
}

nav_msgs::OccupancyGrid::ConstPtr VelodyneOccupancyMapROS::getSensedOccupancyMap()
{
    return _latest_local_map;
}

Eigen::Isometry3d VelodyneOccupancyMapROS::getTransformation(ros::Time time_stamp)
{
    // Get the transformation from /map to /base_link
    geometry_msgs::TransformStamped base_link_T_map_transform = _tfBuffer.lookupTransform("base_link",
                                                                                          "map",
                                                                                          time_stamp,
                                                                                          ros::Duration(1.0));
    return tf2::transformToEigen(base_link_T_map_transform);
}

void VelodyneOccupancyMapROS::spin()
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
                grid_map::GridMapRosConverter::toOccupancyGrid(_local_grid_map, _grid_map_layer_name, -100, 100, local_occupancy_grid_map);

                nav_msgs::OccupancyGrid exclusion_zone_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_exclusion_submap, _grid_map_layer_name, -100, 100, exclusion_zone_occupancy_grid_map);

                _navigation_map_pub.publish(local_occupancy_grid_map);
                _blind_zone_pub.publish(exclusion_zone_occupancy_grid_map);
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
