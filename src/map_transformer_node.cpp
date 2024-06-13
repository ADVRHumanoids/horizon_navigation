#include <map_transformer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_transformer_node");
    ros::NodeHandle nh;

    double map_width, map_height, blind_zone_width, blind_zone_height;
    int rate;

    ros::param::param<int>("~rate", rate, 100);
    ROS_INFO("running at rate: %d", rate);

    ros::param::param<double>("~map_width", map_width, 10);
    ROS_INFO("map width: %f", map_width);

    ros::param::param<double>("~map_height", map_height, 10);
    ROS_INFO("map height: %f", map_height);

    ros::param::param<double>("~blind_zone_width", blind_zone_width, 1);
    ROS_INFO("blind zone width: %f", blind_zone_width);

    ros::param::param<double>("~blind_zone_height", blind_zone_height, 1);
    ROS_INFO("blind zone height: %f", blind_zone_height);

    ros::Rate r(rate);


    auto mt = MapTransformer(map_width, map_height, blind_zone_width, blind_zone_height);

    while(ros::ok())
    {
        mt.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}