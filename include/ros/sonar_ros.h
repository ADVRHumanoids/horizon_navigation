#ifndef SONAR_OCCUPANCY_MAP_ROS_H
#define SONAR_OCCUPANCY_MAP_ROS_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>
#include <sonar.h>


class SonarOccupancyMapROS
{

public:

    struct SonarConfig
    {
        std::string sensor_name;
        std::string topic_name;
        double detection_range = 0.5;
        double arc_resolution = 30;
    };

    typedef std::shared_ptr<SonarOccupancyMapROS> Ptr;

    SonarOccupancyMapROS();
    void spin();
    void update();
    grid_map::GridMap getMap();
    static std::vector<SonarOccupancyMapROS::SonarConfig> load_param_file(ros::NodeHandle nh);
    std::map<std::string, sensor_msgs::Range> getRanges();
    SonarOccupancyMap::Ptr getSonarOccupancyMap();


private:

    void init_publishers();
    void init_subscribers();
    void init_transform();
    void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);

    std::map<std::string, ros::Subscriber> _range_subs;
    ros::Publisher _sonar_map_publisher;
    ros::NodeHandle _nh, _nhpr;

    ros::Rate _rate;

    SonarOccupancyMap::Ptr _sonar_occupancy_map;

    std::map<std::string, SonarOccupancyMap::Sonar::Ptr> _sensors;
    std::map<std::string, std::string> _sensor_topics;

    std::map<std::string, sensor_msgs::Range> _range_msgs;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    YAML::Node _config;
    std::vector<std::string> _sensor_topic_names;

    grid_map::GridMap _map;

};

#endif
