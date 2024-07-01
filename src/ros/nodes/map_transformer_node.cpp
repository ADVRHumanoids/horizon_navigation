#include <ros/map_transformer_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_transformer_node");

    double rate;
    ros::param::param<double>("~rate", rate, 100);
    ROS_INFO("Running at rate: %f", rate);

    auto mt = MapTransformerROS(rate);
    mt.spin();

    return 0;
}
