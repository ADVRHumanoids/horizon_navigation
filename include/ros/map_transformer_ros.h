#ifndef MAP_TRANSFORMER_NODE_H
#define MAP_TRANSFORMER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <map_transformer.h>

class MapTransformerROS
{

public:

    MapTransformerROS(double rate);
    void spin();
    bool update();
    grid_map::GridMap getMap();
    grid_map::GridMap getBlindZone();

private:

    bool load_params();
    void init_subscribers();
    void init_publishers();

    void localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

    ros::NodeHandle _nh, _nhpr;

    ros::Rate _rate;

    double _map_width, _map_height, _blind_zone_width, _blind_zone_height;

    nav_msgs::OccupancyGrid::ConstPtr _latest_local_map;

    grid_map::GridMap _grid_map, _exclusion_submap;

    ros::Subscriber _local_map_sub;
    ros::Publisher _transformed_local_pub, _blind_zone_pub;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    std::string _grid_map_layer_name;

    MapTransformer::Ptr _map_transformer;


};

#endif
