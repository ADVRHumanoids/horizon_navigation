#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sonar.h>

using namespace grid_map;


class SonarRos
{

public:

    SonarRos(std::vector<std::string> sensor_topic_names);
    void spin();

private:

    void init_publishers();
    void init_subscribers();
    void init_transform();
    void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);

    std::map<std::string, ros::Subscriber> _range_subs;
    ros::Publisher _sonar_map_publisher;
    ros::NodeHandle _nh;

    ros::Rate _rate;

    SonarOccupancyMap::Ptr _sonar_occupancy_map;

    std::map<std::string, Eigen::Isometry3d> _origin_T_sensor_dict;

    std::map<std::string, sensor_msgs::Range::ConstPtr> _range_msgs;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    std::vector<std::string> _sensor_topic_names;

};