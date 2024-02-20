#ifndef OBSTACLESGENERATOR_H
#define OBSTACLESGENERATOR_H

 #include <Eigen/Dense>
// #include <numeric>
// #include <vector>
// #include <iostream>
#include <memory>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <obstacle.h>
// #include <stdexcept>
// #include <unordered_set>
//#include <any>
//#include <variant>
//#include <functional>

// divide ros from computing
class ObstacleGenerator
{

public:

    typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> OccupancyMatrix;


    ObstacleGenerator(double rate, int grid_height, int grid_width, double grid_resolution);
    typedef std::shared_ptr<ObstacleGenerator> Ptr;

    bool addObstacle(Obstacle::Ptr obstacle);
    void clearObstacles();
    void run();

    void add_obstacle_viz(Eigen::Vector3d origin, Eigen::Vector3d radius); //std_msgs::ColorRGBA color;
    void _obstacles_from_occupacy_grid();

    std::vector<Obstacle::Ptr> getObstacles();

private:

    void _occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void _init_subscribers(std::string topic_name);
    void _init_publishers();

    static std_msgs::ColorRGBA _get_default_color();

    bool _compare_distance(const Obstacle::Ptr a, const Obstacle::Ptr b);



    OccupancyMatrix _occupancy_matrix;
    int _grid_height;
    int _grid_width;
    double _grid_resolution;
    Eigen::Vector3d _grid_origin;

    int _obstacle_counter;
    std::vector<Obstacle::Ptr> _obstacles;

    ros::NodeHandle _nh;
    double _rate;

    ros::Subscriber _occupancy_grid_subscriber;
    ros::Publisher _obstacle_publisher;

    visualization_msgs::MarkerArray _obstacle_markers;


};

#endif
