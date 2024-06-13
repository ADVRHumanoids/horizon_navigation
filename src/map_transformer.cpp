#include <map_transformer.h>
#include <chrono>

MapTransformer::MapTransformer(double world_map_width, double world_map_height, double blind_zone_width, double blind_zone_height):
    _tfBuffer(),
    _tfListener(_tfBuffer),
    _map_width(world_map_width),
    _map_height(world_map_height),
    _blind_zone_width(blind_zone_width),
    _blind_zone_height(blind_zone_height)
{

//    _global_map_sub = _nh.subscribe("/global_map", 1, &MapTransformer::globalMapCallback, this);
    _local_map_sub = _nh.subscribe("/local_map", 1, &MapTransformer::localMapCallback, this);
    _world_map_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/transformed_map", 1);

    auto local_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/local_map", _nh);

    // initialize world map
    _world_map.header.frame_id = "map";

    _world_map.info.origin.position.x = - world_map_width / 2;
    _world_map.info.origin.position.y = - world_map_height / 2;
    _world_map.info.origin.position.z = 0;
    _world_map.info.origin.orientation.x = 0;
    _world_map.info.origin.orientation.y = 0;
    _world_map.info.origin.orientation.z = 0;
    _world_map.info.origin.orientation.w = 1;

    _world_map.info.resolution = local_map->info.resolution; //local_map->info.resolution;

    _world_map.info.width = static_cast<unsigned int>(world_map_width / _world_map.info.resolution);
    _world_map.info.height = static_cast<unsigned int>(world_map_width / _world_map.info.resolution);
    _world_map.data.resize(_world_map.info.width * _world_map.info.height, -1);

}

void MapTransformer::localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    _latest_local_map = map_msg;
}

//void MapTransformer::globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
//    _latest_global_map = map_msg;
//}


void MapTransformer::update()
{
    if (_latest_local_map) {
        try
        {
            // Get the transformation from /map to /base_link
            geometry_msgs::TransformStamped base_link_T_map = _tfBuffer.lookupTransform("base_link",
                                                                                        "map",
                                                                                        _latest_local_map->header.stamp,
                                                                                        ros::Duration(1.0));

            _world_map = transformAndFilter(_world_map, *_latest_local_map, base_link_T_map, _blind_zone_width, _blind_zone_height);
            _world_map_pub.publish(_world_map);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }
}

nav_msgs::OccupancyGrid MapTransformer::transformAndFilter(nav_msgs::OccupancyGrid map_a,
                                             const nav_msgs::OccupancyGrid& map_b,
                                             const geometry_msgs::TransformStamped& transformStamped,
                                             double patch_width,
                                             double patch_height)
{

    // transform map_b into map_a coordinate, update map_a with the obtained data except the blind zone

    nav_msgs::OccupancyGrid map_a_updated = map_a;

    // Define the center of the square in the base_link frame (assuming the center of the map)
    unsigned int center_x = map_b.info.width / 2;
    unsigned int center_y = map_b.info.height / 2;

    unsigned int patch_width_cells = static_cast<unsigned int>(patch_width / map_b.info.resolution);
    unsigned int patch_height_cells = static_cast<unsigned int>(patch_height / map_b.info.resolution);

    // Calculate the bounds of the patch
    int exclude_start_x = static_cast<int>(center_x) - static_cast<int>(patch_width_cells / 2);
    int exclude_start_y = static_cast<int>(center_y) - static_cast<int>(patch_height_cells / 2);
    int exclude_end_x = static_cast<int>(center_x) + static_cast<int>(patch_width_cells / 2);
    int exclude_end_y = static_cast<int>(center_y) + static_cast<int>(patch_height_cells / 2);


    // Convert the transform to a tf2::Transform
    tf2::Transform tf_transform;
    tf2::fromMsg(transformStamped.transform, tf_transform);

    // Iterate over the local map data and update the global map
    for (unsigned int b_y = 0; b_y < map_b.info.height; b_y++)
    {
        for (unsigned int b_x = 0; b_x < map_b.info.width; b_x++)
        {
            // Exclude the blind zone from the update
            if (b_x >= exclude_start_x && b_x < exclude_end_x && b_y >= exclude_start_y && b_y < exclude_end_y)
            {
                continue;
            }

            double w_x = b_x * map_b.info.resolution + map_b.info.origin.position.x;
            double w_y = b_y * map_b.info.resolution + map_b.info.origin.position.y;

            // Create a point in the a_map center frame
            tf2::Vector3 world_a_point(w_x, w_y, 0);

            // Transform to the global map frame
            tf2::Vector3 world_b_point = tf_transform.inverse() * world_a_point;

            int a_x = static_cast<int>((world_b_point.x() - map_a.info.origin.position.x) / map_a.info.resolution);
            int a_y = static_cast<int>((world_b_point.y() - map_a.info.origin.position.y) / map_a.info.resolution);


            // Check if the coordinates are within the bounds of the global map
            if (a_x >= 0 && a_x < static_cast<int>(map_a.info.width) && a_y >= 0 && a_y < static_cast<int>(map_a.info.height))
            {
                int b_index = b_x + b_y * map_b.info.width;
                int a_index = a_x + a_y * map_a.info.width;
                map_a_updated.data[a_index] = map_b.data[b_index];
            }
         }
    }

    return map_a_updated;
}

//nav_msgs::OccupancyGrid MapTransformer::additiveUpdate(nav_msgs::OccupancyGrid input_map,
//                                                       const nav_msgs::OccupancyGrid& updating_map,
//                                                       const geometry_msgs::TransformStamped& transformStamped,
//                                                       double patch_width,
//                                                       double patch_height)
//{


//    // Convert the transform to a tf2::Transform
//    tf2::Transform tf_transform;
//    tf2::fromMsg(transformStamped.transform, tf_transform);

//    // Define the center of the square in the base_link frame (assuming the center of the map)
//    unsigned int center_x = input_map.info.width / 2;
//    unsigned int center_y = input_map.info.height / 2;

//    unsigned int patch_width_cells = static_cast<unsigned int>(patch_width / input_map.info.resolution);
//    unsigned int patch_height_cells = static_cast<unsigned int>(patch_height / input_map.info.resolution);

//    // Calculate the bounds of the square
//    int start_x = static_cast<int>(center_x) - static_cast<int>(patch_width_cells / 2);
//    int start_y = static_cast<int>(center_y) - static_cast<int>(patch_height_cells / 2);
//    int end_x = static_cast<int>(center_x) + static_cast<int>(patch_width_cells / 2);
//    int end_y = static_cast<int>(center_y) + static_cast<int>(patch_height_cells / 2);

//    // Iterate over the transformed map data and fill it based on the original map
//    for (unsigned int y = start_y; y < end_y; y++)
//    {
//        for (unsigned int x = start_x; x < end_x; x++)
//        {
//            // Calculate world coordinates of the transformed map cell
//            double world_x = x * input_map.info.resolution + input_map.info.origin.position.x;
//            double world_y = y * input_map.info.resolution + input_map.info.origin.position.y;

//            // Create a point in the base_link frame
//            tf2::Vector3 base_link_point(world_x, world_y, 0);

//            // Transform to the map frame
//            tf2::Vector3 map_point = tf_transform.inverse() * base_link_point;

//            // Convert world coordinates to map coordinates
//            int mx = static_cast<int>((map_point.x() - updating_map.info.origin.position.x) / updating_map.info.resolution);
//            int my = static_cast<int>((map_point.y() - updating_map.info.origin.position.y) / updating_map.info.resolution);

//            // Check if the coordinates are within the bounds of the original map
//            if (mx >= 0 && mx < static_cast<int>(updating_map.info.width) && my >= 0 && my < static_cast<int>(updating_map.info.height))
//            {
//                int map_index = mx + my * updating_map.info.width;
//                int transformed_index = x + y * input_map.info.width;

//                input_map.data[transformed_index] = updating_map.data[map_index];
//            }
//        }
//    }

//    return input_map;
//}

//nav_msgs::OccupancyGrid MapTransformer::subtractiveUpdate(nav_msgs::OccupancyGrid input_map,
//                                                          const nav_msgs::OccupancyGrid& updating_map,
//                                                          const geometry_msgs::TransformStamped& transformStamped,
//                                                          double patch_width,
//                                                          double patch_height)
//{
//    // Convert the transform to a tf2::Transform
//    tf2::Transform tf_transform;
//    tf2::fromMsg(transformStamped.transform, tf_transform);

//    // Define the center of the square in the base_link frame (assuming the center of the map)
//    unsigned int center_x = input_map.info.width / 2;
//    unsigned int center_y = input_map.info.height / 2;

//    unsigned int patch_width_cells = static_cast<unsigned int>(patch_width / input_map.info.resolution);
//    unsigned int patch_height_cells = static_cast<unsigned int>(patch_height / input_map.info.resolution);

//    // Calculate the bounds of the patch
//    int start_x = static_cast<int>(center_x) - static_cast<int>(patch_width_cells / 2);
//    int start_y = static_cast<int>(center_y) - static_cast<int>(patch_height_cells / 2);
//    int end_x = static_cast<int>(center_x) + static_cast<int>(patch_width_cells / 2);
//    int end_y = static_cast<int>(center_y) + static_cast<int>(patch_height_cells / 2);

//    // Iterate over the transformed map data
//    for (unsigned int y = 0; y < input_map.info.height; y++)
//    {
//        for (unsigned int x = 0; x < input_map.info.width; x++)
//        {
//            // Skip the square area
//            if (x >= start_x && x < end_x && y >= start_y && y < end_y)
//            {
//                continue;
//            }

//            // Calculate world coordinates of the transformed map cell
//            double world_x = x * input_map.info.resolution + input_map.info.origin.position.x;
//            double world_y = y * input_map.info.resolution + input_map.info.origin.position.y;

//            // Create a point in the base_link frame
//            tf2::Vector3 base_link_point(world_x, world_y, 0);

//            // Transform to the map frame
//            tf2::Vector3 map_point = tf_transform.inverse() * base_link_point;

//            // Convert world coordinates to map coordinates
//            int mx = static_cast<int>((map_point.x() - updating_map.info.origin.position.x) / updating_map.info.resolution);
//            int my = static_cast<int>((map_point.y() - updating_map.info.origin.position.y) / updating_map.info.resolution);

//            // Check if the coordinates are within the bounds of the original map
//            if (mx >= 0 && mx < static_cast<int>(updating_map.info.width) && my >= 0 && my < static_cast<int>(updating_map.info.height))
//            {
//                int map_index = mx + my * updating_map.info.width;
//                int transformed_index = x + y * input_map.info.width;

//                input_map.data[transformed_index] = updating_map.data[map_index];
//            }
//        }
//    }

//    return input_map;
//}

//nav_msgs::OccupancyGrid MapTransformer::bOverA(const nav_msgs::OccupancyGrid& map_a,
//                                               const nav_msgs::OccupancyGrid& map_b,
//                                               const geometry_msgs::TransformStamped& transformStamped,
//                                               double patch_width,
//                                               double patch_height)
//{

//    nav_msgs::OccupancyGrid map_a_updated = map_a;

//    // Define the center of the square in the base_link frame (assuming the center of the map)
//    unsigned int center_x = map_b.info.width / 2;
//    unsigned int center_y = map_b.info.height / 2;

//    unsigned int patch_width_cells = static_cast<unsigned int>(patch_width / map_b.info.resolution);
//    unsigned int patch_height_cells = static_cast<unsigned int>(patch_height / map_b.info.resolution);

//    // Calculate the bounds of the patch
//    int exclude_start_x = static_cast<int>(center_x) - static_cast<int>(patch_width_cells / 2);
//    int exclude_start_y = static_cast<int>(center_y) - static_cast<int>(patch_height_cells / 2);
//    int exclude_end_x = static_cast<int>(center_x) + static_cast<int>(patch_width_cells / 2);
//    int exclude_end_y = static_cast<int>(center_y) + static_cast<int>(patch_height_cells / 2);

//    // Convert the transform to a tf2::Transform
//    tf2::Transform tf_transform;
//    tf2::fromMsg(transformStamped.transform, tf_transform);

//    // Iterate over the local map data and update the global map
//    for (unsigned int b_y = 0; b_y < map_b.info.height; b_y++)
//    {
//        for (unsigned int b_x = 0; b_x < map_b.info.width; b_x++)
//        {

//            if (b_x >= exclude_start_x && b_x < exclude_end_x && b_y >= exclude_start_y && b_y < exclude_end_y)
//            {
//                continue;
//            }

//            double w_x = b_x * map_b.info.resolution + map_b.info.origin.position.x;
//            double w_y = b_y * map_b.info.resolution + map_b.info.origin.position.y;

////            // Create a point in the a_map center frame
//            tf2::Vector3 world_a_point(w_x, w_y, 0);

//            // Transform to the global map frame
//            tf2::Vector3 world_b_point = tf_transform.inverse() * world_a_point;

//            int a_x = static_cast<int>((world_b_point.x() - map_a.info.origin.position.x) / map_a.info.resolution);
//            int a_y = static_cast<int>((world_b_point.y() - map_a.info.origin.position.y) / map_a.info.resolution);


//            // Check if the coordinates are within the bounds of the global map
//            if (a_x >= 0 && a_x < static_cast<int>(map_a.info.width) && a_y >= 0 && a_y < static_cast<int>(map_a.info.height))
//            {
//                int b_index = b_x + b_y * map_b.info.width;
//                int a_index = a_x + a_y * map_a.info.width;
//                map_a_updated.data[a_index] = map_b.data[b_index];
//            }
//         }
//    }

//    return map_a_updated;
//}

//nav_msgs::OccupancyGrid MapTransformer::filterLocal(const nav_msgs::OccupancyGrid& global_map,
//                                                        const nav_msgs::OccupancyGrid& local_map,
//                                                        const geometry_msgs::TransformStamped& transformStamped,
//                                                        double patch_width,
//                                                        double patch_height)
//{

//    nav_msgs::OccupancyGrid updated_global_map = global_map;

//    // Define the center of the square in the base_link frame (assuming the center of the map)
//    unsigned int center_x = local_map.info.width / 2;
//    unsigned int center_y = local_map.info.height / 2;

//    unsigned int patch_width_cells = static_cast<unsigned int>(patch_width / local_map.info.resolution);
//    unsigned int patch_height_cells = static_cast<unsigned int>(patch_height / local_map.info.resolution);

//    // Calculate the bounds of the patch
//    int exclude_start_x = static_cast<int>(center_x) - static_cast<int>(patch_width_cells / 2);
//    int exclude_start_y = static_cast<int>(center_y) - static_cast<int>(patch_height_cells / 2);
//    int exclude_end_x = static_cast<int>(center_x) + static_cast<int>(patch_width_cells / 2);
//    int exclude_end_y = static_cast<int>(center_y) + static_cast<int>(patch_height_cells / 2);

//    // Convert the transform to a tf2::Transform
//    tf2::Transform tf_transform;
//    tf2::fromMsg(transformStamped.transform, tf_transform);

//    // Iterate over the local map data and update the global map
//    for (unsigned int y = 0; y < local_map.info.height; y++)
//    {
//        for (unsigned int x = 0; x < local_map.info.width; x++)
//        {

//            if (x >= exclude_start_x && x < exclude_end_x && y >= exclude_start_y && y < exclude_end_y)
//            {
//                continue;
//            }

//            // Calculate world coordinates of the local map cell
//            double wx = x * local_map.info.resolution + local_map.info.origin.position.x;
//            double wy = y * local_map.info.resolution + local_map.info.origin.position.y;

//            // Create a point in the base_link frame
//            tf2::Vector3 local_map_point(wx, wy, 0);

//            // Transform to the global map frame
//            tf2::Vector3 global_map_point = tf_transform * local_map_point;

//            // Convert world coordinates to global map coordinates
//            int gx = static_cast<int>((global_map_point.x() - global_map.info.origin.position.x) / global_map.info.resolution);
//            int gy = static_cast<int>((global_map_point.y() - global_map.info.origin.position.y) / global_map.info.resolution);

//            // Check if the coordinates are within the bounds of the global map
//            if (gx >= 0 && gx < static_cast<int>(global_map.info.width) && gy >= 0 && gy < static_cast<int>(global_map.info.height))
//            {
//                int global_index = gx + gy * global_map.info.width;
//                int local_index = x + y * local_map.info.width;
//                updated_global_map.data[global_index] = local_map.data[local_index];

//            }
//        }
//    }

//    return updated_global_map;
//}

//nav_msgs::OccupancyGrid MapTransformer::bFilterA(const nav_msgs::OccupancyGrid &map_a,
//                                                           const nav_msgs::OccupancyGrid &map_b,
//                                                           double patch_width,
//                                                           double patch_height)
//{
//    nav_msgs::OccupancyGrid filtered_map = map_a;

//    // Define the center of the square in the base_link frame (assuming the center of the map)
//    unsigned int center_x = map_a.info.width / 2;
//    unsigned int center_y = map_a.info.height / 2;

//    unsigned int patch_width_cells = static_cast<unsigned int>(patch_width / map_a.info.resolution);
//    unsigned int patch_height_cells = static_cast<unsigned int>(patch_height / map_a.info.resolution);

//    // Calculate the bounds of the patch
//    int exclude_start_x = static_cast<int>(center_x) - static_cast<int>(patch_width_cells / 2);
//    int exclude_start_y = static_cast<int>(center_y) - static_cast<int>(patch_height_cells / 2);
//    int exclude_end_x = static_cast<int>(center_x) + static_cast<int>(patch_width_cells / 2);
//    int exclude_end_y = static_cast<int>(center_y) + static_cast<int>(patch_height_cells / 2);

//    for (unsigned int y = 0; y < map_a.info.height; y++)
//    {
//        for (unsigned int x = 0; x < map_a.info.width; x++)
//        {
//            int index = x + y * map_a.info.width;
//            if (x >= exclude_start_x && x < exclude_end_x && y >= exclude_start_y && y < exclude_end_y)
//            {
//                continue;
//            }
//            else
//            {
//                if (map_b.data[index] < 50)
//                {
//                    filtered_map.data[index] = map_b.data[index];
//                }
//            }
//        }
//    }

//    return filtered_map;
//}
