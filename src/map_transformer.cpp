#include <map_transformer.h>
#include <chrono>

MapTransformer::MapTransformer(double map_width,
                               double map_height,
                               double blind_zone_width,
                               double blind_zone_height,
                               std::string map_layer_name)
:
    _blind_zone_width(blind_zone_width),
    _blind_zone_height(blind_zone_height),
    _map_origin(0.0, 0.0),
    _map_layer_name(map_layer_name)
{

    _grid_map.add(_map_layer_name);
    _grid_map.setPosition(_map_origin);
    _grid_map.setFrameId("map");
    _grid_map.setGeometry(grid_map::Length(map_width, map_height), 0.01);

    _local_grid_map.add(_map_layer_name);
    _local_grid_map.setPosition(_map_origin);
    _local_grid_map.setFrameId("base_link");
    _local_grid_map.setGeometry(grid_map::Length(map_width, map_height), 0.01);

    // Create the exclusion submap
    grid_map::Length exclusion_size(_blind_zone_width, _blind_zone_height);
    grid_map::Position map_center = _grid_map.getPosition();
    grid_map::Index submapStartIndex;
    bool isSuccess;

    _exclusion_submap = _grid_map.getSubmap(map_center, exclusion_size, submapStartIndex, isSuccess);

}

void MapTransformer::update(const nav_msgs::OccupancyGrid occupancy_grid, const Eigen::Isometry3d transform)
{

    // move world map to follow the base_link of the robot
    grid_map::Position base_link_T_map_position(transform.inverse().translation().x(),
                                                transform.inverse().translation().y());

    _grid_map.move(base_link_T_map_position);
    _exclusion_submap.move(base_link_T_map_position);

    // filter the world map with the exclusion submap
    filterMap(_grid_map, occupancy_grid, _exclusion_submap, transform);


    // transform the world map into the local frame "base_link"
    _local_grid_map = _grid_map.getTransformedMap(transform,
                                                  _map_layer_name,
                                                  "base_link");

    // select only the submap of given lenght
    bool success;
    _local_grid_map = _local_grid_map.getSubmap(_local_grid_map.getPosition(), _grid_map.getLength(), success);

}

grid_map::GridMap MapTransformer::getMap()
{
    return _local_grid_map;
}

grid_map::GridMap MapTransformer::getBlindZone()
{
    return _exclusion_submap;
}

void MapTransformer::filterMap(grid_map::GridMap& map,
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

            // Transform to world map
            grid_map::Position3 cellPosition3d(cellX, cellY, 0);
            auto world_cell_position = transform.inverse() * cellPosition3d;

            grid_map::Position cellPosition(world_cell_position.x(), world_cell_position.y());

            if (map.isInside(cellPosition))
            {

                if (exclusionSubmap.isInside(cellPosition))
                {
                    continue; // Skip this cell as it is within the exclusion submap
                }

                int occupancyValue = occupancyGrid.data[j * occupancyGrid.info.width + i];
                map.atPosition(_map_layer_name, cellPosition) = occupancyValue;
            }

        }
    }
}
