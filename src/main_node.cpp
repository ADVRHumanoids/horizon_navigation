#include <ros/ros.h>
#include <obstacles_generator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacles_node");
    ros::NodeHandle nh(""), nhpr("~");

    int rate;
    if (!nhpr.hasParam("rate"))
    {
        rate = 100;
    }
    else
    {
        nhpr.getParam("rate", rate);
    }

    std::cout << "running rate at " << rate << " Hz" << std::endl;

    auto obs_gen = ObstacleGenerator(nh, rate, 60, 60, 0.1);

//    Eigen::Vector3d origin_1; origin_1 << 0, 0, 0;
//    Eigen::Vector3d radius_1; radius_1 << 1, 1, 1;

//    Eigen::Vector3d origin_2; origin_2 << 5, 0, 0;
//    Eigen::Vector3d radius_2; radius_2 << 1, 1, 1;

//    obs_gen.add_obstacle_viz(origin_1, radius_1);
//    obs_gen.add_obstacle_viz(origin_2, radius_2);

    ros::Rate r(rate);
    while(ros::ok())
    {
        obs_gen.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
