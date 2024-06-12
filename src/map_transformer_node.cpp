#include <map_transformer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_transformer_node");
    ros::NodeHandle nh;

    double width;
    double height;
    int rate;

    ros::param::param<int>("~rate", rate, 10);
    ROS_INFO("running at rate: %d", rate);

    ros::param::param<double>("~width", width, 10);
    ROS_INFO("map width: %f", width);

    ros::param::param<double>("~height", height, 10);
    ROS_INFO("map height: %f", height);

    ros::Rate r(rate);


    auto mt = MapTransformer(width, height);

    while(ros::ok())
    {

        mt.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
