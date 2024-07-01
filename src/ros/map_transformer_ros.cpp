#include <ros/map_transformer_ros.h>

MapTransformerROS::MapTransformerROS(double rate):
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


    _map_transformer = std::make_unique<MapTransformer>(_map_width,
                                                        _map_height,
                                                        _blind_zone_width,
                                                        _blind_zone_height,
                                                        _grid_map_layer_name);


}

bool MapTransformerROS::load_params()
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

void MapTransformerROS::init_subscribers()
{
    _local_map_sub = _nh.subscribe("/local_map", 1, &MapTransformerROS::localMapCallback, this);
//    auto msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/local_map", _nh);

}

void MapTransformerROS::init_publishers()
{
    _transformed_local_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/filtered_local_map", 1);
    _blind_zone_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/blind_zone", 1);
}


void MapTransformerROS::localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    _latest_local_map = map_msg;
}

bool MapTransformerROS::update()
{
    if (_latest_local_map)
    {

        // Get the transformation from /map to /base_link
        geometry_msgs::TransformStamped base_link_T_map_transform = _tfBuffer.lookupTransform("base_link",
                                                                                              "map",
                                                                                              _latest_local_map->header.stamp,
                                                                                              ros::Duration(1.0));
        auto base_link_T_map =  tf2::transformToEigen(base_link_T_map_transform);



        _map_transformer->update(*_latest_local_map, base_link_T_map);

        _grid_map = _map_transformer->getMap();
        _exclusion_submap = _map_transformer->getBlindZone();

        return true;
    }

    return false;



}

grid_map::GridMap MapTransformerROS::getMap()
{
    return _grid_map;
}

grid_map::GridMap MapTransformerROS::getBlindZone()
{
    return _exclusion_submap;
}

void MapTransformerROS::spin()
{

    while (_nh.ok())
    {
        try
        {
            auto success = update();

            if (success)
            {
            // publish grid_maps to ros occupancy maps
                nav_msgs::OccupancyGrid world_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_grid_map, _grid_map_layer_name, -100, 100, world_occupancy_grid_map);

                nav_msgs::OccupancyGrid exclusion_zone_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_exclusion_submap, _grid_map_layer_name, -100, 100, exclusion_zone_occupancy_grid_map);

                _transformed_local_pub.publish(world_occupancy_grid_map);
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
