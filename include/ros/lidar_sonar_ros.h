#ifndef LIDAR_SONAR_ROS_H
#define LIDAR_SONAR_ROS_H

#include <ros/velodyne_ros.h>
#include <ros/sonar_ros.h>

class LidarSonarROS
{

public:

    LidarSonarROS(double rate);
    void spin();
    bool update();


private:



    ros::NodeHandle _nh, _nhpr;

    ros::Rate _rate;

    VelodyneOccupancyMapROS::Ptr _velodyne_ros;
    SonarOccupancyMapROS::Ptr _sonar_ros;

    nav_msgs::OccupancyGrid::ConstPtr _latest_local_map;

    grid_map::GridMap _local_sonar_grid_map;

    ros::Publisher _local_sonar_map_publisher, _velodyne_map_publisher;



};

#endif
