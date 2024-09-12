#ifndef OBSTACLESGENERATOR_H
#define OBSTACLESGENERATOR_H

 #include <Eigen/Dense>
// #include <numeric>
// #include <vector>
// #include <iostream>
#include <yaml-cpp/yaml.h>
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


    ObstacleGenerator(double grid_height, 
                      double grid_width, 
                      double grid_resolution, 
                      std::vector<std::string> topic_name = {"/map"},
                      std::string rviz_markers_topic_name = "");
                      
    typedef std::shared_ptr<ObstacleGenerator> Ptr;

    bool addObstacle(Obstacle::Ptr obstacle);
    void clearObstacles();
    void run();

    void addObstacleViz(int id, Eigen::Vector3d origin, Eigen::Vector3d radius, std_msgs::ColorRGBA color);
    void obstaclesFromOccupacyGrid(const OccupancyMatrix occupancy_matrix);

    std::vector<Obstacle::Ptr> getObstacles();

    bool setObstacleRadius(double radius);
    bool setMaxObstacleNum(int max_obstacle_num);
    bool setAngleThreshold(double angle);
    void setBlindAngle(double min_angle, double max_angle);

    double getObstacleRadius();
    int getMaxObstacleNum();
    double getAngleThreshold();

    void _update(); // should be private

private:

    void _occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg, const std::string& matrix);
    void _init_subscribers(std::vector<std::string> topic_name);
    void _init_publishers(std::string topic_name);
    void _init_load_config();



//    void _filter_angle();

    static std_msgs::ColorRGBA _get_default_color();
    bool _min_distance(const Obstacle::Ptr a, const Obstacle::Ptr b);
    bool _max_distance(const Obstacle::Ptr a, const Obstacle::Ptr b);
    double _compute_distance(const Obstacle::Ptr obs);
    std::vector<Obstacle::Ptr> _sort_angle_distance();
    void setBlindsight();
    void visualizeObstaclesViz();



    std::map<std::string, OccupancyMatrix> _occupancy_matrices;
    int _grid_height_cells;
    int _grid_width_cells;
    double _grid_resolution;
    Eigen::Vector3d _grid_origin;


    int _max_obstacle_num;
    double _radius_obstacle;
    double _angle_threshold;

    double _min_angle, _max_angle;

    std::vector<std::string> _occupancy_grid_topic_names;

    YAML::Node _config;

//    int _obstacle_counter;
    std::vector<Obstacle::Ptr> _obstacles;

    ros::NodeHandle _nh, _nhpr;
    std::map<std::string, ros::Subscriber> _occupancy_grid_subscribers;
    ros::Publisher _obstacle_publisher;

    std::string _publisher_name;

    visualization_msgs::MarkerArray _obstacle_markers;


};

#endif
