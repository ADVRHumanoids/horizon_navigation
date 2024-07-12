#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <velodyne.h>
#include <sonar.h>
#include <sensor_msgs/Range.h>


class Navigation
{
public:

    typedef std::shared_ptr<Navigation> Ptr;

    Navigation(VelodyneOccupancyMap::Ptr velodyne_occupancy_map,
                   SonarOccupancyMap::Ptr sonar_occupancy_map);

    bool update(const nav_msgs::OccupancyGrid occupancy_grid,
                const Eigen::Isometry3d transform,
                std::map<std::string, sensor_msgs::Range::ConstPtr> range_messages);

    grid_map::GridMap getVelodyneLocalMap();
    grid_map::GridMap getSonarLocalMap();

private:


    void filterMap(grid_map::GridMap& map,
                   const nav_msgs::OccupancyGrid occupancyGrid,
                   const grid_map::GridMap& exclusionSubmap,
                   const Eigen::Isometry3d transform);



    VelodyneOccupancyMap::Ptr _velodyne_occupancy_map;
    SonarOccupancyMap::Ptr _sonar_occupancy_map;

    grid_map::GridMap _global_grid_map, _local_grid_map, _exclusion_submap, _local_sonar_grid_map;


};

#endif // MAP_TRANSFORMER_H
