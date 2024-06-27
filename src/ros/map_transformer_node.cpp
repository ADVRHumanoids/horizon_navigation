#include <ros/map_transformer_node.h>

MapTransformerRos::MapTransformerRos(double rate):
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

bool MapTransformerRos::load_params()
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

void MapTransformerRos::init_subscribers()
{
    _local_map_sub = _nh.subscribe("/local_map", 1, &MapTransformerRos::localMapCallback, this);
}

void MapTransformerRos::init_publishers()
{
    _transformed_local_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/filtered_local_map", 1);
    _blind_zone_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/blind_zone", 1);
}


void MapTransformerRos::localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    _latest_local_map = map_msg;
}

void MapTransformerRos::spin()
{

    while (_nh.ok())
    {

        if (_latest_local_map)
        {
            try
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

                // publish grid_maps to ros occupancy maps
                nav_msgs::OccupancyGrid world_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_grid_map, _grid_map_layer_name, -100, 100, world_occupancy_grid_map);

                nav_msgs::OccupancyGrid exclusion_zone_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_exclusion_submap, _grid_map_layer_name, -100, 100, exclusion_zone_occupancy_grid_map);

                _transformed_local_pub.publish(world_occupancy_grid_map);
                _blind_zone_pub.publish(exclusion_zone_occupancy_grid_map);

            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }

        _rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_transformer_node");

    double rate;
    ros::param::param<double>("~rate", rate, 100);
    ROS_INFO("Running at rate: %f", rate);

    auto mt = MapTransformerRos(rate);
    mt.spin();

    return 0;
}
