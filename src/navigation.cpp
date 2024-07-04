#include <navigation.h>

Navigation::Navigation(VelodyneOccupancyMap::Ptr velodyne_occupancy_map,
                       SonarOccupancyMap::Ptr sonar_occupancy_map)
    :
      _velodyne_occupancy_map(velodyne_occupancy_map),
      _sonar_occupancy_map(sonar_occupancy_map)
{

    _global_grid_map = _velodyne_occupancy_map->getGlobalMap();
    _exclusion_submap = velodyne_occupancy_map->getBlindZone();

    _global_grid_map.add("sonar", 0.0);

}

bool Navigation::update(const nav_msgs::OccupancyGrid occupancy_grid, const Eigen::Isometry3d transform, std::map<std::string, sensor_msgs::Range::ConstPtr> range_msgs)
{

    grid_map::Position base_link_T_map_position(transform.inverse().translation().x(),
                                                transform.inverse().translation().y());

    for (auto range_msg : range_msgs)
    {
        if (range_msg.second)
        {
            _sonar_occupancy_map->setData(range_msg.first,
                                          range_msg.second->header.frame_id,
                                          range_msg.second->range,
                                          range_msg.second->min_range,
                                          range_msg.second->max_range,
                                          range_msg.second->field_of_view);

            _sonar_occupancy_map->update();
        }
    }

    _sonar_grid_map = _sonar_occupancy_map->getMap();
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


    // set a reset zone with the sonars
    grid_map::Length reset_zone_length(1.2 + 0.6, 0.8 + 0.6);
    grid_map::Index reset_zone_size(reset_zone_length.x() / _global_grid_map.getResolution(), reset_zone_length.y() / _global_grid_map.getResolution());

    // get the starting positon and the index
    grid_map::Position reset_zone_start_pos = _global_grid_map.getPosition() + grid_map::Position(reset_zone_length.x() / 2.0, reset_zone_length.y() / 2.0);
    grid_map::Index reset_zone_start_index;

    _global_grid_map.getIndex(reset_zone_start_pos, reset_zone_start_index);

    for (grid_map::SubmapIterator iterator(_global_grid_map, reset_zone_start_index, reset_zone_size); !iterator.isPastEnd(); ++iterator)
    {

        // transform from world to base link
        grid_map::Position global_map_position;
        _global_grid_map.getPosition(*iterator, global_map_position);
        auto local_map_position = transform * grid_map::Position3(global_map_position.x(), global_map_position.y(), 0.0);

        grid_map::Index sonar_index;
        _sonar_grid_map.getIndex(grid_map::Position(local_map_position.x(), local_map_position.y()), sonar_index);


        if (_global_grid_map.at("obstacles", *iterator) > 50.0)
        {
//                        std::cout << sonar_map.at("sonar_map", sonar_index) << std::endl;
            if (_sonar_grid_map.at("sonar_map", sonar_index) != 100.0)
            {
                _global_grid_map.at("obstacles", *iterator) = 0.0;
            }
        }

        if (_sonar_grid_map.at("sonar_map", sonar_index) > 50.0)
        {
            _global_grid_map.at("obstacles", *iterator) = _sonar_grid_map.at("sonar_map", sonar_index);
        }
    }

    _local_grid_map = _global_grid_map.getTransformedMap(transform,
                                                         "obstacles",
                                                         "base_link");


//    _local_grid_map.get("obstacles") += _global_grid_map.getTransformedMap(transform,
//                                                                           "sonar",
//                                                                           "base_link").get("sonar");



    return true;

}

grid_map::GridMap Navigation::getLocalMap()
{
    return _local_grid_map;
}

