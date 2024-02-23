#include <obstacles_generator.h>

ObstacleGenerator::ObstacleGenerator(double rate, int grid_height, int grid_width, double grid_resolution):
    _rate(rate),
    _grid_height(grid_height),
    _grid_width(grid_width),
    _grid_resolution(grid_resolution),
    _grid_origin(0, 0, 0)
{

    _nh = ros::NodeHandle("");
    _nhpr = ros::NodeHandle("~");

    _occupancy_matrix.resize(grid_height, grid_width);
    _occupancy_matrix.setZero();

    _max_obstacle_num = 10;
    _radius_obstacle = grid_resolution;


    _init_publishers();

    std::string occupancy_grid_topic_name = "/costmap_node/costmap/costmap";
    _init_subscribers(occupancy_grid_topic_name);

    _init_load_config();
}

void ObstacleGenerator::_init_load_config()
{
    if(!_nhpr.hasParam("config"))
    {
        std::cout << "Missing 'config' ros parameter for obstacle_generator node, using default \n" <<  std::endl;
    }
    else
    {
        std::string config_string;
        _nhpr.getParam("config", config_string);
        _config = YAML::Load(config_string);

        if (!_config["max_obstacle_num"])
        {
            std::cout << "Missing 'max_obstacle_num' for rviz, using default (" << _max_obstacle_num << ") \n" << std::endl;
        }
        else
        {
            std::cout << "Setting 'max_obstacle_num' for rviz to " << _config["max_obstacle_num"] << "\n" << std::endl;
            _max_obstacle_num = _config["max_obstacle_num"].as<int>();
        }

        if (!_config["radius_obstacle"])
        {
            std::cout << "Missing 'radius_obstacle', using default (" << _radius_obstacle << ") \n" << std::endl;
        }
        else
        {
            std::cout << "Setting 'radius_obstacle' to " << _config["radius_obstacle"] << "\n" << std::endl;
            _radius_obstacle = _config["radius_obstacle"].as<int>();
        }
    }
}

bool ObstacleGenerator::addObstacle(Obstacle::Ptr obstacle)
{
    _obstacles.push_back(std::move(obstacle));
    return true;
}

void ObstacleGenerator::clearObstacles()
{
    _obstacles.clear();
}

void ObstacleGenerator::_init_subscribers(std::string topic_name)
{
    _occupancy_grid_subscriber = _nh.subscribe(topic_name, 10, &ObstacleGenerator::_occupancy_grid_callback, this);
}

void ObstacleGenerator::_init_publishers()
{
    _obstacle_publisher = _nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10);
}

void ObstacleGenerator::add_obstacle_viz(int id, Eigen::Vector3d origin, Eigen::Vector3d radius, std_msgs::ColorRGBA color = _get_default_color()) //std_msgs::ColorRGBA color = _get_default_color()
{
    auto obstacle_marker = visualization_msgs::Marker();
    obstacle_marker.header.frame_id = "base_link";
    obstacle_marker.header.stamp = ros::Time::now();
    obstacle_marker.ns = "sphere";
    obstacle_marker.id = id;
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
    obstacle_marker.color.a = color.a; //color.a;
    obstacle_marker.color.r = color.r; //color.r;
    obstacle_marker.color.g = color.g; //color.g;
    obstacle_marker.color.b = color.b; //color.b;
//    obstacle_marker.lifetime = ros::Duration(1/_rate);
    _obstacle_markers.markers.push_back(obstacle_marker);

}

void ObstacleGenerator::_obstacles_from_occupacy_grid()
{
    // Iterating over the matrix
//    _obstacle_counter = 0;

    for (int elem_w = 0; elem_w < _grid_width; ++elem_w)
    {
        for (int elem_h = 0; elem_h < _grid_height; ++elem_h)
        {
            if(_occupancy_matrix(elem_h, elem_w) > 0)
            {
               // origin is at the center of the grid
               double grid_origin_x = (_grid_height * _grid_resolution) / 2;
               double grid_origin_y = (_grid_width * _grid_resolution) / 2;
               // + _grid_resolution: apparently the grid is offsetted by ONE UNIT of _grid_resolution, both in x and y
               double x = elem_h * _grid_resolution - grid_origin_x + _grid_resolution;
               double y = elem_w * _grid_resolution - grid_origin_y + _grid_resolution;

               Eigen::Vector3d obstacle_origin;
               Eigen::Vector3d obstacle_radius;

               obstacle_origin << x, y, 0;
               obstacle_radius << _radius_obstacle, _radius_obstacle, _radius_obstacle;

               auto cas_obs = std::make_shared<SphereObstacle>(obstacle_origin, obstacle_radius);
               addObstacle(cas_obs);
            }
        }
    }

}

std::vector<Obstacle::Ptr> ObstacleGenerator::getObstacles()
{
    return _obstacles;
}

bool ObstacleGenerator::setObstacleRadius(double radius)
{
    _radius_obstacle = radius;
    std::cout << "setting radius to: " << _radius_obstacle << std::endl;
    return true;
}

bool ObstacleGenerator::setMaxObstacleNum(int max_obstacle_num)
{
    _max_obstacle_num = max_obstacle_num;
    std::cout << "setting max num obstacles to: " << _max_obstacle_num << std::endl;
    return true;
}

void ObstacleGenerator::run()
{
    // fill obstacles from sensors
    // clear markers at every cycle
    _obstacle_markers.markers.clear(); // remove from here

    clearObstacles();
    // compute obstacles from occupancy grid coming from laserscan
    _obstacles_from_occupacy_grid();

    // sort the vector of obstacle from closest to farther
    std::sort(_obstacles.begin(), _obstacles.end(),
              [this](auto a, auto b)
    {
        return _compare_distance(a, b);
    });

//     visualize obstacle
    auto id_obs = 0;
    std_msgs::ColorRGBA color_marker;
    for (auto elem : _obstacles)
    {

        if (auto obs = std::dynamic_pointer_cast<SphereObstacle>(elem))
        {
            if (id_obs < _max_obstacle_num)
            {
                color_marker.r = 0.0;
                color_marker.g = 1.0;
                color_marker.b = 0.0; // Yellow
                color_marker.a = 1.0;

            }
            else
            {
                color_marker.r = 1.0;
                color_marker.g = 1.0;
                color_marker.b = 0.0;
                color_marker.a = 1.0;
            }
                add_obstacle_viz(id_obs, obs->getOrigin(), obs->getRadius(), color_marker);
                id_obs++;
        }
    }

    std::cout << "number of obstacles founds: " << id_obs << std::endl;
//    std::cout << "number of obstacles in vector: " << _obstacles.size() << std::endl;

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

bool ObstacleGenerator::_compare_distance(const Obstacle::Ptr a, const Obstacle::Ptr b)
{

    auto distance_a = (_grid_origin - a->getOrigin()).norm();
    auto distance_b = (_grid_origin - b->getOrigin()).norm();

    return distance_a < distance_b;

}

void ObstacleGenerator::_occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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




