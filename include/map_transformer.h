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

    typedef std::shared_ptr<MapTransformer> Ptr;

    MapTransformer(double map_width,
                   double map_height,
                   double blind_zone_width,
                   double blind_zone_height,
                   std::string map_layer_name);

    void update(nav_msgs::OccupancyGrid occupancy_grid,
                const Eigen::Isometry3d transform);


    grid_map::GridMap getMap();
    grid_map::GridMap getBlindZone();

private:

    grid_map::GridMap getExclusionSubmap(const grid_map::GridMap& map,
                                         const grid_map::Position& center,
                                         const grid_map::Length& size);

    void filterMap(grid_map::GridMap& map,
                   const nav_msgs::OccupancyGrid occupancyGrid,
                   const grid_map::GridMap& exclusionSubmap,
                   const Eigen::Isometry3d transform);


    grid_map::GridMap _grid_map, _exclusion_submap, _local_grid_map;
    grid_map::Position _map_origin, _sensor_origin;

    double _blind_zone_width, _blind_zone_height;  // Width of the world map and the exclusion zone in meters

    std::string _map_layer_name;
};

#endif // MAP_TRANSFORMER_H
