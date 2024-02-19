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

// #include <stdexcept>
// #include <unordered_set>
//#include <any>
//#include <variant>
//#include <functional>

class ObstacleGenerator
{

public:

    typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> OccupancyMatrix;


    ObstacleGenerator(ros::NodeHandle nh, double rate, int grid_height, int grid_width, double grid_resolution);
    typedef std::shared_ptr<ObstacleGenerator> Ptr;

    bool addObstacle(Eigen::Vector3d origin, Eigen::Vector3d radius);
    void run();

    void add_obstacle_viz(Eigen::Vector3d origin, Eigen::Vector3d radius); //std_msgs::ColorRGBA color;

    void _obstacles_from_occupacy_grid();

private:

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void _init_subscribers(std::string topic_name);
    void _init_publishers();

    static std_msgs::ColorRGBA _get_default_color();



    OccupancyMatrix _occupancy_matrix;
    int _grid_height;
    int _grid_width;
    double _grid_resolution;

    int _obstacle_counter;

    ros::NodeHandle _nh;
    double _rate;

    ros::Subscriber _occupancy_grid_subscriber;
    ros::Publisher _obstacle_publisher;

    visualization_msgs::MarkerArray _obstacle_markers;


};

#endif
