#include <velodyne.h>

VelodyneOccupancyMap::VelodyneOccupancyMap(Eigen::Vector2d map_origin = Eigen::Vector2d::Zero(),
                                           Eigen::Vector2d map_length = Eigen::Vector2d::Constant(4.0),
                                           Eigen::Vector2d blind_zone_length = Eigen::Vector2d::Constant(1.0),
                                           double map_resolution = 0.01,
                                           std::string map_layer_name = "main")
    :
    _map_origin(map_origin),
    _map_length(map_length),
    _blind_zone_length(blind_zone_length),
    _map_resolution(map_resolution),
    _map_layer_name(map_layer_name)
{
    _world_frame_id = "map";
    _robot_frame_id = "base_link";

    // Create the global map
    _global_grid_map.add(_map_layer_name);
    _global_grid_map.setPosition(_map_origin);
    _global_grid_map.setFrameId(_world_frame_id);
    _global_grid_map.setGeometry(_map_length, _map_resolution);

    // Create the local map
    _local_grid_map.add(_map_layer_name);
    _local_grid_map.setPosition(_map_origin);
    _local_grid_map.setFrameId(_robot_frame_id);
    _local_grid_map.setGeometry(_map_length, _map_resolution);

    // Create the exclusion submap
    _exclusion_submap.add(_map_layer_name);
    _exclusion_submap.setPosition(_map_origin);
    _exclusion_submap.setFrameId(_robot_frame_id);
    _exclusion_submap.setGeometry(_blind_zone_length, _map_resolution);

    // set up info from sonar
//    _sonar_handler = std::make_unique<SonarOccupancyMap>(_local_grid_map.getPosition());
}

void VelodyneOccupancyMap::filterMap(grid_map::GridMap& map,
                           const nav_msgs::OccupancyGrid occupancyGrid,
                           const grid_map::GridMap& exclusionSubmap,
                           const Eigen::Isometry3d transform)
{
    // Loop through the occupancy grid and update the grid map

    for (int i = 0; i < occupancyGrid.info.width; ++i)
    {
        for (int j = 0; j < occupancyGrid.info.height; ++j)
        {
            // Calculate the real-world position of the cell
            double cellX = occupancyGrid.info.origin.position.x + i * occupancyGrid.info.resolution;
            double cellY = occupancyGrid.info.origin.position.y + j * occupancyGrid.info.resolution;

            // grid map in base_link frame
            grid_map::Position cell_position(cellX, cellY);
            if (exclusionSubmap.isInside(cell_position))
            {
                continue; // Skip this cell as it is within the exclusion submap
            }

            // Transform to world map
            grid_map::Position3 cellPosition3d(cellX, cellY, 0);
            auto world_cell_position = transform.inverse() * cellPosition3d;

            grid_map::Position cellPosition(world_cell_position.x(), world_cell_position.y());

            if (map.isInside(cellPosition))
            {
                int occupancyValue = occupancyGrid.data[j * occupancyGrid.info.width + i];
                map.atPosition(_map_layer_name, cellPosition) = occupancyValue;
            }

        }
    }
}

bool VelodyneOccupancyMap::update(const nav_msgs::OccupancyGrid occupancy_grid, const Eigen::Isometry3d transform)
{

    // move world map to follow the base_link of the robot
    grid_map::Position base_link_T_map_position(transform.inverse().translation().x(),
                                                transform.inverse().translation().y());

    _global_grid_map.move(base_link_T_map_position);

    // filter the world map with the exclusion submap (w.r.t. base_link frame)
    filterMap(_global_grid_map, occupancy_grid, _exclusion_submap, transform);


    // transform the world map into the local frame "base_link"
    _local_grid_map = _global_grid_map.getTransformedMap(transform,
                                                  _map_layer_name,
                                                  _robot_frame_id);


    return true;

}

grid_map::GridMap VelodyneOccupancyMap::getLocalMap()
{
    return _local_grid_map;
}

grid_map::GridMap VelodyneOccupancyMap::getGlobalMap()
{
    return _global_grid_map;
}

grid_map::GridMap VelodyneOccupancyMap::getBlindZone()
{
    return _exclusion_submap;
}


