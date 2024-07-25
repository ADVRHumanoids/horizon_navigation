#include <navigation.h>

Navigation::Navigation(VelodyneOccupancyMap::Ptr velodyne_occupancy_map,
                       SonarOccupancyMap::Ptr sonar_occupancy_map)
    :
      _velodyne_occupancy_map(velodyne_occupancy_map),
      _sonar_occupancy_map(sonar_occupancy_map)
{

    _global_grid_map = _velodyne_occupancy_map->getGlobalMap();
    _exclusion_submap = velodyne_occupancy_map->getBlindZone();
//    _sonar_grid_map = sonar_occupancy_map->getMap();

    _global_grid_map.add("sonar", 0.0);


}

bool Navigation::update(const nav_msgs::OccupancyGrid occupancy_grid, const Eigen::Isometry3d transform, std::map<std::string, sensor_msgs::Range> range_msgs)
{

    grid_map::Position base_link_T_map_position(transform.inverse().translation().x(),
                                                transform.inverse().translation().y());

    for (auto range_msg : range_msgs)
    {
        _sonar_occupancy_map->setData(range_msg.first,
                                      range_msg.second.header.frame_id,
                                      range_msg.second.range,
                                      range_msg.second.min_range,
                                      range_msg.second.max_range,
                                      range_msg.second.field_of_view);
    }


    _global_grid_map.clear("sonar");

    for (auto sensor_name : _sonar_occupancy_map->getSensorsNames())
    {
        _sonar_occupancy_map->updateGridMapWithSonar(_global_grid_map, sensor_name, "sonar", transform.inverse());
    }

    // ============================================================================

    _global_grid_map.move(base_link_T_map_position);

    // filter the world map with the exclusion submap (w.r.t. base_link frame)
    for (int i = 0; i < occupancy_grid.info.width; ++i)
    {
        for (int j = 0; j < occupancy_grid.info.height; ++j)
        {
            // Calculate the real-world position of the cell
            double cellX = occupancy_grid.info.origin.position.x + i * occupancy_grid.info.resolution;
            double cellY = occupancy_grid.info.origin.position.y + j * occupancy_grid.info.resolution;

            // grid map in base_link frame
            grid_map::Position cell_position(cellX, cellY);
            if (_exclusion_submap.isInside(cell_position))
            {
                continue; // Skip this cell as it is within the exclusion submap
            }

            // Transform to world map
            grid_map::Position3 cellPosition3d(cellX, cellY, 0);
            auto world_cell_position = transform.inverse() * cellPosition3d;

            grid_map::Position cellPosition(world_cell_position.x(), world_cell_position.y());

            if (_global_grid_map.isInside(cellPosition))
            {
                int occupancyValue = occupancy_grid.data[j * occupancy_grid.info.width + i];
                _global_grid_map.atPosition("obstacles", cellPosition) = occupancyValue;
            }

        }
    }

    for (auto sonar : _sonar_occupancy_map->getSensorsNames())
    {
        if (!_sonar_occupancy_map->getSensorsStatus(sonar))
        {
            _sonar_occupancy_map->clearSonarCone(_global_grid_map, sonar, "obstacles", transform.inverse());
        }
    }


    _local_grid_map = _global_grid_map.getTransformedMap(transform,
                                                         "obstacles",
                                                         "base_link");

    _local_sonar_grid_map = _global_grid_map.getTransformedMap(transform,
                                                               "sonar",
                                                               "base_link");




    return true;

}

grid_map::GridMap Navigation::getVelodyneLocalMap()
{
    return _local_grid_map;
}

grid_map::GridMap Navigation::getSonarLocalMap()
{
    return _local_sonar_grid_map;
}

grid_map::GridMap Navigation::getBlindZone()
{
    return _exclusion_submap;
}

