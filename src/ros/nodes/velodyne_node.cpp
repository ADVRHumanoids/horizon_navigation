#include <ros/velodyne_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_node");

    double rate;
    ros::param::param<double>("~rate", rate, 100);
    ROS_INFO("Running at rate: %f", rate);

    auto mt = VelodyneOccupancyMapROS(rate);
    mt.spin();

    return 0;
}
