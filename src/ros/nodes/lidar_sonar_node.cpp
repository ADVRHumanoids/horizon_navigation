#include <ros/lidar_sonar_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_sonar_node");

    double rate;
    ros::param::param<double>("~rate", rate, 500);
    ROS_INFO("Running at rate: %f", rate);

    auto ls = LidarSonarROS(rate);
    ls.spin();

    return 0;
}
