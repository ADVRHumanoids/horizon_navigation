#include <map_transformer.h>
#include <chrono>

MapTransformer::MapTransformer(double map_width,
                               double map_height,
                               double blind_zone_width,
                               double blind_zone_height)
:
    _tfBuffer(),
    _tfListener(_tfBuffer),
    _blind_zone_width(blind_zone_width),
    _blind_zone_height(blind_zone_height),
    _map_origin(0.0, 0.0)
{

    _grid_map.add("obstacle_layer");
    _grid_map.setPosition(_map_origin);
    _grid_map.setFrameId("map");
    _grid_map.setGeometry(grid_map::Length(map_width, map_height), 0.01);

//    _global_map_sub = _nh.subscribe("/global_map", 1, &MapTransformer::globalMapCallback, this);
    _local_map_sub = _nh.subscribe("/local_map", 1, &MapTransformer::localMapCallback, this);

    _transformed_local_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/filtered_local_map", 1);
    _blind_zone_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/blind_zone", 1);


    // Create the exclusion submap
    grid_map::Length exclusion_size(_blind_zone_width, _blind_zone_height);
    grid_map::Position map_center = _grid_map.getPosition();
    grid_map::Index submapStartIndex;
    bool isSuccess;

    _exclusion_submap = _grid_map.getSubmap(map_center, exclusion_size, submapStartIndex, isSuccess);



}

void MapTransformer::localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    _latest_local_map = map_msg;
}

void MapTransformer::update()
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

            // move world map to follow the base_link of the robot
            grid_map::Position base_link_T_map_position(base_link_T_map.inverse().translation().x(), base_link_T_map.inverse().translation().y());

            _grid_map.move(base_link_T_map_position);
            _exclusion_submap.move(base_link_T_map_position);

            // filter the world map with the exclusion submap
            filterMap(_grid_map, *_latest_local_map, _exclusion_submap, base_link_T_map);

            // publish grid_maps to ros occupancy maps
            nav_msgs::OccupancyGrid world_occupancy_grid_map;
            grid_map::GridMapRosConverter::toOccupancyGrid(_grid_map, "obstacle_layer", -100, 100, world_occupancy_grid_map);
            _transformed_local_pub.publish(world_occupancy_grid_map);

            nav_msgs::OccupancyGrid exclusion_zone_occupancy_grid_map;
            grid_map::GridMapRosConverter::toOccupancyGrid(_exclusion_submap, "obstacle_layer", -100, 100, exclusion_zone_occupancy_grid_map);
            _blind_zone_pub.publish(exclusion_zone_occupancy_grid_map);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }
}

void MapTransformer::filterMap(grid_map::GridMap& map,
                               const nav_msgs::OccupancyGrid& occupancyGrid,
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
                map.atPosition("obstacle_layer", cellPosition) = occupancyValue;
            }

        }
    }
}
