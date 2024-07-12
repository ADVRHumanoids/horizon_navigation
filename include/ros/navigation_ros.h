#ifndef NAVIGATION_ROS_H
#define NAVIGATION_ROS_H

#include <ros/velodyne_ros.h>
#include <ros/sonar_ros.h>
#include <navigation.h>

class NavigationROS
{

public:

    NavigationROS(double rate);
    void spin();
    bool update();


private:



    ros::NodeHandle _nh, _nhpr;

    ros::Rate _rate;

    ros::Publisher _map_pub;


    VelodyneOccupancyMapROS::Ptr _velodyne_ros;
    SonarOccupancyMapROS::Ptr _sonar_ros;

    Navigation::Ptr _navigation;

    grid_map::GridMap _local_grid_map, _local_sonar_grid_map;

    ros::Publisher _local_map_publisher, _local_sonar_map_publisher;



};

#endif
