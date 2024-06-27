#ifndef SONAR_ROS_H
#define SONAR_ROS_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>
#include <sonar.h>


class SonarRos
{

public:

    SonarRos();
    void spin();

private:

    bool load_param_file();
    void init_publishers();
    void init_subscribers();
    void init_transform();
    void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);

    std::map<std::string, ros::Subscriber> _range_subs;
    ros::Publisher _sonar_map_publisher;
    ros::NodeHandle _nh, _nhpr;

    ros::Rate _rate;

    SonarOccupancyMap::Ptr _sonar_occupancy_map;

    std::map<std::string, SonarOccupancyMap::Sonar> _sensors;

    std::map<std::string, std::string> _sensor_topics;

    std::map<std::string, sensor_msgs::Range::ConstPtr> _range_msgs;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    YAML::Node _config;
    std::vector<std::string> _sensor_topic_names;

};

#endif
