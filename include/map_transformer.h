#ifndef MAP_TRANSFORMER_H
#define MAP_TRANSFORMER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>


class MapTransformer
{
public:

    MapTransformer(double map_width,
                   double map_height,
                   double blind_zone_width,
                   double blind_zone_height);

    void update();

private:

    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    nav_msgs::OccupancyGrid transformAndFilter(nav_msgs::OccupancyGrid a_map,
                                 const nav_msgs::OccupancyGrid& b_map,
                                 const tf2::Transform& transformStamped,
                                 double patch_width,
                                 double patch_height);

    grid_map::GridMap getExclusionSubmap(const grid_map::GridMap& map,
                                         const grid_map::Position& center,
                                         const grid_map::Length& size);

    void filterMap(grid_map::GridMap& map,
                   const nav_msgs::OccupancyGrid& occupancyGrid,
                   const grid_map::GridMap& exclusionSubmap,
                   const Eigen::Isometry3d transform);

    YAML::Node _config;

    grid_map::GridMap _grid_map, _exclusion_submap;
    grid_map::Position _map_origin, _sensor_origin;

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _local_map_sub; // _global_map_sub,
    ros::Publisher _transformed_local_pub, _blind_zone_pub;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    nav_msgs::OccupancyGrid::ConstPtr _latest_local_map; // _latest_global_map
    double _blind_zone_width, _blind_zone_height;  // Width of the world map and the exclusion zone in meters
};

#endif // MAP_TRANSFORMER_H
