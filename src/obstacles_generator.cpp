#include <obstacles_generator.h>

ObstacleGenerator::ObstacleGenerator(ros::NodeHandle nh, double rate, int grid_height, int grid_width, double grid_resolution):
    _nh(nh),
    _rate(rate),
    _grid_height(grid_height),
    _grid_width(grid_width),
    _grid_resolution(grid_resolution),
    _obstacle_counter(0)
{
    _occupancy_matrix.resize(grid_height, grid_width);

    _init_publishers();

    std::string occupancy_grid_topic_name = "/costmap_node/costmap/costmap";
    _init_subscribers(occupancy_grid_topic_name);
}

bool ObstacleGenerator::addObstacle(Eigen::Vector3d origin, Eigen::Vector3d radius)
{
    return true;
}

void ObstacleGenerator::_init_subscribers(std::string topic_name)
{
    _occupancy_grid_subscriber = _nh.subscribe(topic_name, 10, &ObstacleGenerator::occupancyGridCallback, this);
}

void ObstacleGenerator::_init_publishers()
{
    _obstacle_publisher = _nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10);
}

void ObstacleGenerator::add_obstacle_viz(Eigen::Vector3d origin, Eigen::Vector3d radius) //std_msgs::ColorRGBA color = _get_default_color()
{
    auto obstacle_marker = visualization_msgs::Marker();
    obstacle_marker.header.frame_id = "base_link";
    obstacle_marker.header.stamp = ros::Time::now();
    obstacle_marker.ns = "sphere";
    obstacle_marker.id = _obstacle_counter;
    obstacle_marker.type = visualization_msgs::Marker::SPHERE;
    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.pose.position.x = origin[0];
    obstacle_marker.pose.position.y = origin[1];
    obstacle_marker.pose.position.z = origin[2];
    obstacle_marker.pose.orientation.x = 0.0;
    obstacle_marker.pose.orientation.y = 0.0;
    obstacle_marker.pose.orientation.z = 0.0;
    obstacle_marker.pose.orientation.w = 1.0;
    obstacle_marker.scale.x = 2 * radius[0];
    obstacle_marker.scale.y = 2 * radius[1];
    obstacle_marker.scale.z = 2 * radius[2];
    obstacle_marker.color.a = 1.; //color.a;
    obstacle_marker.color.r = 1.; //color.r;
    obstacle_marker.color.g = 1.; //color.g;
    obstacle_marker.color.b = 0.; //color.b;
//    obstacle_marker.lifetime = ros::Duration(1/_rate);
    _obstacle_markers.markers.push_back(obstacle_marker);

}

void ObstacleGenerator::_obstacles_from_occupacy_grid()
{
    // Iterating over the matrix
    _obstacle_counter = 0;

    for (int elem_w = 0; elem_w < _grid_width; ++elem_w)
    {
        for (int elem_h = 0; elem_h < _grid_height; ++elem_h)
        {
            if(_occupancy_matrix(elem_h, elem_w) > 0)
            {
               double x = elem_h * _grid_resolution - (_grid_height * _grid_resolution) / 2;
               double y = elem_w * _grid_resolution - (_grid_width * _grid_resolution) / 2;

               std::cout << "found obstacle at: [" << x << ", " << y << "] " << std::endl;

               Eigen::Vector3d obstacle_origin;
               Eigen::Vector3d obstacle_radius;
               obstacle_origin << x, y, 0;
               obstacle_radius << _grid_resolution, _grid_resolution, _grid_resolution;

               add_obstacle_viz(obstacle_origin, obstacle_radius);

               _obstacle_counter += 1;
            }
        }
    }

}

void ObstacleGenerator::run()
{
    // fill obstacles from sensors
    _obstacle_markers.markers.clear(); // remove from here
    _obstacles_from_occupacy_grid();
    std::cout << "number of obstacle founds: " << _obstacle_counter << std::endl;

    // publish obstacles
    _obstacle_publisher.publish(_obstacle_markers);
}

std_msgs::ColorRGBA ObstacleGenerator::_get_default_color()
{
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0; // Yellow
    color.a = 1.0;

    return color;
}

void ObstacleGenerator::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // Extract grid data from the message
    int width = msg->info.width;
    int height = msg->info.height;

    if (width != _grid_width || height != _grid_width) {
        ROS_ERROR("Received grid size does not match expected size. Skipping processing.");
    }

    // Copy grid data into the matrix
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = y * width + x;
            _occupancy_matrix(x, y) = msg->data[index];
        }
    }
}




