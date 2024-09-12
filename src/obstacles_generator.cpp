#include <obstacles_generator.h>

ObstacleGenerator::ObstacleGenerator(double grid_height, 
                                     double grid_width, 
                                     double grid_resolution, 
                                     std::vector<std::string> input_topic_names,
                                     std::string rviz_markers_topic_name):
    _grid_resolution(grid_resolution),
    _grid_origin(0, 0, 0),
    _min_angle(0.0),
    _max_angle(0.0)
{
    _nh = ros::NodeHandle("");
    _nhpr = ros::NodeHandle("~");


    _grid_height_cells = static_cast<int>(grid_height / _grid_resolution); // from meters to grid units
    _grid_width_cells = static_cast<int>(grid_width / _grid_resolution); // from meters to grid units

    std::cout << "Grid height: " << _grid_height_cells << std::endl;
    std::cout << "Grid width: " << _grid_width_cells << std::endl;


    // initializing occupancy matrices
    for (auto topic_name : input_topic_names)
    {
        _occupancy_grid_topic_names.push_back(topic_name);
        _occupancy_matrices[topic_name].resize(_grid_height_cells, _grid_width_cells);
        _occupancy_matrices[topic_name].setZero();
    }

    _radius_obstacle = _grid_resolution;
    _angle_threshold = _radius_obstacle;
    _max_obstacle_num = 2 * M_PI / _angle_threshold; // technically, it's the circle divided by the angle treshold

    _publisher_name = _occupancy_grid_topic_names[0] + "/obstacles";
    if (!rviz_markers_topic_name.empty())
    {
        _publisher_name = rviz_markers_topic_name;
    }
    _init_publishers(_publisher_name);

    // name of topic from which the occupancy map is taken
//    std::string occupancy_grid_topic_name = "/map";
    _init_subscribers(_occupancy_grid_topic_names);

    _init_load_config();
}

void ObstacleGenerator::_init_load_config()
{
    if(!_nhpr.hasParam("config"))
    {
        std::cout << "Missing 'config' ros parameter for '" << _nhpr.getNamespace() << "' node, using default \n" <<  std::endl;
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

void ObstacleGenerator::setBlindsight()
{
    // Remove elements that meet the condition
    auto cone = [this](Obstacle::Ptr obstacle, double min_angle, double max_angle)
    {
        double angle = obstacle->getAngle();
        return angle >= min_angle && angle <= max_angle;
    };

    auto circle = [this](Obstacle::Ptr obstacle, Eigen::Vector2d origin, double radius)
    {
        Eigen::Vector2d obs_origin(obstacle->getOrigin().x(), obstacle->getOrigin().y());
        double distance_squared = (obs_origin - origin).squaredNorm();
        double radius_squared = radius * radius;

        return distance_squared < radius_squared;
    };

    auto ellipse = [this](Obstacle::Ptr obstacle, Eigen::Vector2d origin, Eigen::Matrix2d axes)
    {
        Eigen::Vector2d obs_origin(obstacle->getOrigin().x(), obstacle->getOrigin().y());
        Eigen::Vector2d diff = obs_origin - origin;
        double distance = diff.transpose() * axes.inverse() * diff;

        return distance <= 1.0;
    };

    Eigen::Vector2d origin_blindzone(1.0, 0.0);
    // circle
    double radius_blindzone = 0.5;

//    ellipse

    Eigen::Vector2d axes(0.5, 0.5);
    Eigen::Matrix2d axes_blindzone;
    axes_blindzone << axes[0]*axes[0], 0,
                      0,               axes[1]*axes[1];

    // remove obstacles in blind angle
    _obstacles.erase(std::remove_if(_obstacles.begin(),
                                    _obstacles.end(),
//                                    std::bind(circle, std::placeholders::_1, origin_blindzone, radius_blindzone)
//                                    std::bind(cone, std::placeholders::_1, _min_angle, _max_angle)
                                    std::bind(ellipse, std::placeholders::_1, origin_blindzone, axes_blindzone)
                                    ),

                     _obstacles.end());
}

void ObstacleGenerator::_init_subscribers(std::vector<std::string> topic_names)
{

    for (const auto& topic_name : topic_names)
    {

        ros::Subscriber sub = _nh.subscribe<nav_msgs::OccupancyGrid>(topic_name,
                                                                     10,
                                                                     std::bind(&ObstacleGenerator::_occupancy_grid_callback, this, std::placeholders::_1, topic_name)
                                                                     );

        _occupancy_grid_subscribers[topic_name] = sub;
    }
}

void ObstacleGenerator::_init_publishers(std::string topic_name)
{
    _obstacle_publisher = _nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
}

void ObstacleGenerator::addObstacleViz(int id, Eigen::Vector3d origin, Eigen::Vector3d radius, std_msgs::ColorRGBA color = _get_default_color()) //std_msgs::ColorRGBA color = _get_default_color()
{
    auto obstacle_marker = visualization_msgs::Marker();
    obstacle_marker.header.frame_id = "base_link";
    obstacle_marker.header.stamp = ros::Time::now();
    obstacle_marker.ns = "sphere"; //_publisher_name;
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
    obstacle_marker.color.a = color.a;
    obstacle_marker.color.r = color.r;
    obstacle_marker.color.g = color.g;
    obstacle_marker.color.b = color.b;

    _obstacle_markers.markers.push_back(obstacle_marker);

}

void ObstacleGenerator::visualizeObstaclesViz()
{

    // remove all markers visualization at each loop
    if (!_obstacle_markers.markers.empty())
    {
        visualization_msgs::MarkerArray delete_markers;
        auto delete_marker = visualization_msgs::Marker();
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_markers.markers.push_back(delete_marker);
        _obstacle_publisher.publish(delete_markers);
    }

    _obstacle_markers.markers.clear(); // remove from here

    //  visualize obstacle
    auto id_obs = 0;
    std_msgs::ColorRGBA color_marker;
    for (auto elem : _obstacles)
    {

        if (auto obs = std::dynamic_pointer_cast<SphereObstacle>(elem))
        {
            if (id_obs < _max_obstacle_num)
            {
                // Green
                color_marker.r = 0.0;
                color_marker.g = 1.0;
                color_marker.b = 0.0;
                color_marker.a = 1.0;

            }
            else
            {
                // Yellow
                color_marker.r = 1.0;
                color_marker.g = 1.0;
                color_marker.b = 0.0;
                color_marker.a = 0.1;
            }

//                auto discretized_angle = static_cast<int>(obs->getAngle() / _angle_threshold);
//                auto normalized_angle = static_cast<double>(discretized_angle + 15) / 30;
//                color_marker.r = std::max(0.0, 1.0 - normalized_angle); // Red component
//                color_marker.g = 1.0 - std::abs(normalized_angle - 0.5) * 2; // Green component
//                color_marker.b = std::max(0.0, normalized_angle - 0.5) * 2; // Blue component
//                color_marker.a = 1.0;

//                std::cout << "addin obstacle [" << discretized_angle << "] with colors: " << std::endl;
//                std::cout << color_marker.r << std::endl;
//                std::cout << color_marker.g << std::endl;
//                std::cout << color_marker.b << std::endl;

            addObstacleViz(id_obs, obs->getOrigin(), obs->getRadius(), color_marker);
            id_obs++;
        }
    }

//    std::cout << "number of obstacles founds: " << id_obs << std::endl;

    // publish obstacles
    _obstacle_publisher.publish(_obstacle_markers);
}

void ObstacleGenerator::_update()
{
    // compute obstacles from occupancy grid coming from laserscan
    for (auto occupancy_matrix : _occupancy_matrices)
    {
        obstaclesFromOccupacyGrid(occupancy_matrix.second);
    }

    // remove blind spot given by setBlindAngle
    setBlindsight();

    // sort obstacles by angle: the closest for each obstacle with the same angle from the origin
    _obstacles = _sort_angle_distance();

    // sort obstacles by distance: the closest to the origin
//    std::sort(_obstacles.begin(), _obstacles.end(),
//              [this](auto a, auto b)
//    {
//        return _min_distance(a, b);
//    });

    visualizeObstaclesViz();

}

void ObstacleGenerator::obstaclesFromOccupacyGrid(const OccupancyMatrix occupancy_matrix)
{
    // Iterating over the matrix
//    _obstacle_counter = 0;

    for (int elem_w = 0; elem_w < _grid_width_cells; ++elem_w)
    {
        for (int elem_h = 0; elem_h < _grid_height_cells; ++elem_h)
        {
            if(occupancy_matrix(elem_h, elem_w) == 100)
            {
               // origin is at the center of the grid
               double grid_origin_x = (_grid_height_cells * _grid_resolution) / 2;
               double grid_origin_y = (_grid_width_cells * _grid_resolution) / 2;
               // + _grid_resolution: apparently the grid is offsetted by ONE UNIT of _grid_resolution, both in x and y
               double x = elem_h * _grid_resolution - grid_origin_x + _grid_resolution;
               double y = elem_w * _grid_resolution - grid_origin_y + _grid_resolution;

               Eigen::Vector3d obstacle_origin;
               Eigen::Vector3d obstacle_radius;

               obstacle_origin << x, y, 0;
               obstacle_radius << _radius_obstacle, _radius_obstacle, _radius_obstacle;

//               if (atan2(y, x) < _min_angle || (atan2(y, x) > _max_angle))
//                {
                auto obs = std::make_shared<SphereObstacle>(obstacle_origin, obstacle_radius);
                addObstacle(obs);
//                }
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
    std::cout << "Setting radius to: " << _radius_obstacle << std::endl;
    return true;
}

bool ObstacleGenerator::setMaxObstacleNum(int max_obstacle_num)
{
    _max_obstacle_num = max_obstacle_num;
    std::cout << "Setting max num obstacles to: " << _max_obstacle_num << std::endl;
    return true;
}

bool ObstacleGenerator::setAngleThreshold(double angle)
{
    // discretized angle below which the obstacles' angle get rounded.
    // All obstacles inside a discretized angle gets treated as the same angle
    _angle_threshold = angle;
    std::cout << "Setting angle threshold to: " << _angle_threshold << std::endl;
    return true;

}

void ObstacleGenerator::setBlindAngle(double min_angle, double max_angle)
{
    _min_angle = fmod(min_angle, 2 * M_PI);
    _max_angle = fmod(max_angle, 2 * M_PI);

//    std::cout << "set min: "<< _min_angle << std::endl;
    //    std::cout << "set max: "<< _max_angle << std::endl;
}

double ObstacleGenerator::getObstacleRadius()
{
    return _radius_obstacle;
}

int ObstacleGenerator::getMaxObstacleNum()
{
    return _max_obstacle_num;
}

double ObstacleGenerator::getAngleThreshold()
{
    return _angle_threshold;
}

void ObstacleGenerator::run()
{
    // fill obstacles from sensors
    // clear markers at every cycle
    clearObstacles();
    _update();
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

bool ObstacleGenerator::_max_distance(const Obstacle::Ptr a, const Obstacle::Ptr b)
{

    auto distance_a = _compute_distance(a);
    auto distance_b = _compute_distance(b);

    return distance_a > distance_b;

}

bool ObstacleGenerator::_min_distance(const Obstacle::Ptr a, const Obstacle::Ptr b)
{

    auto distance_a = _compute_distance(a);
    auto distance_b = _compute_distance(b);

    return distance_a < distance_b;

}

double ObstacleGenerator::_compute_distance(const Obstacle::Ptr obs)
{

    auto distance = (_grid_origin - obs->getOrigin()).norm();

    return distance;

}

std::vector<Obstacle::Ptr> ObstacleGenerator::_sort_angle_distance()
{
    std::unordered_map<int, std::vector<Obstacle::Ptr>> slices_obstacle;

    // add element to vector in map depending on angle
    for (const auto& obstacle : _obstacles)
    {
        int discretized_angle = static_cast<int>(obstacle->getAngle() / _angle_threshold);
//        std::cout << "found obstacle at angle: " << obstacle->getAngle() << "[discretized: " << discretized_angle << "]" << std::endl;
        slices_obstacle[discretized_angle].push_back(obstacle);
    }

//    std::cout << "===========================" << std::endl;
    // sort all vector in map
    for (auto& pair : slices_obstacle)
    {
        auto& obs_vec = pair.second;
        // sort the vector of obstacle from closest to farther
        std::sort(obs_vec.begin(), obs_vec.end(),
                  [this](auto a, auto b)
        {
            return _max_distance(a, b);
        });
    }

    // take only the first
    std::vector<Obstacle::Ptr> sorted_obstacles;

    int obstacle_i = 0;
    bool all_empty = false;

    while (!all_empty) //&& obstacle_i < _max_obstacle_num
    {
        all_empty = true;
        for (auto& pair : slices_obstacle)
        {
            auto& obs_vec = pair.second;

            if (!obs_vec.empty())
            {
                sorted_obstacles.push_back(obs_vec.back());
                obs_vec.pop_back();
                obstacle_i++;
                all_empty = false;
            }
        }
    }

    return sorted_obstacles;


}

void ObstacleGenerator::_occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg, const std::string& topic_name)
{

    // Extract grid data from the message
    int width = msg->info.width;
    int height = msg->info.height;

    if (width < _grid_width_cells || height < _grid_height_cells) {
        ROS_INFO("Received grid (%d x %d) is smaller than the expected size (%d x %d). Skipping processing.", width, height, _grid_width_cells, _grid_height_cells);
        return;
    }

    // Calculate the starting indices to center the sub-grid
    int start_x = (width - _grid_width_cells) / 2;
    int start_y = (height - _grid_height_cells) / 2;

    // Copy grid data into the matrix, only within the specified height and width
    for (int y = 0; y < _grid_height_cells; ++y)
    {
        for (int x = 0; x < _grid_width_cells; ++x)
        {
            int index = (start_y + y) * width + (start_x + x);
            _occupancy_matrices[topic_name](x, y) = msg->data[index];
        }
    }
}




