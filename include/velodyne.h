#ifndef VELODYNE_OCCUPANCY_MAP_H
#define VELODYNE_OCCUPANCY_MAP_H

#include <Eigen/Dense>
#include <memory>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <sonar.h>

class VelodyneOccupancyMap
{

public:

    typedef std::shared_ptr<VelodyneOccupancyMap> Ptr;

    VelodyneOccupancyMap(Eigen::Vector2d map_origin,
               Eigen::Vector2d map_length,
               Eigen::Vector2d blind_zone_length,
               double map_resolution,
               std::string map_layer_name);

    bool update(const nav_msgs::OccupancyGrid occupancy_grid, const Eigen::Isometry3d transform);

    void filterMap(grid_map::GridMap& map,
                   const nav_msgs::OccupancyGrid occupancyGrid,
                   const grid_map::GridMap& exclusionSubmap,
                   const Eigen::Isometry3d transform);

    grid_map::GridMap getLocalMap();
    grid_map::GridMap getGlobalMap();
    grid_map::GridMap getBlindZone();

private:

   grid_map::GridMap _global_grid_map, _local_grid_map, _exclusion_submap;

   grid_map::Position _map_origin;
   grid_map::Length _map_length, _blind_zone_length;
   double _map_resolution;
   std::string _world_frame_id, _robot_frame_id, _map_layer_name;




};

#endif
