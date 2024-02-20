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

    auto obs_gen = ObstacleGenerator(rate, 60, 60, 0.1);

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




//ObstacleGeneratorROS::ObstacleGeneratorROS(std::string name, int grid_width, int grid_height, int grid_resolution, std::list<std::string> args)
//{
//    if(ros::ok())
//    {
//        ROS_ERROR("Ros node already initialized with name %s",
//                  ros::this_node::getName().c_str());
//        return false;
//    }

//    std::vector<const char *> args_vec;
//    for(auto& a : args)
//    {
//        args_vec.push_back(a.c_str());
//    }

//    int argc = args_vec.size();

//    char ** argv = (char **)args_vec.data();

//    name += "_cpp";

//    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);

//    ros::NodeHandle nh(""), nhpr("~");

//    int rate;
//    if (!nhpr.hasParam("rate"))
//    {
//        rate = 100;
//    }
//    else
//    {
//        nhpr.getParam("rate", rate);
//    }

//    std::cout << "running rate at " << rate << " Hz" << std::endl;


//    auto obs_gen = ObstacleGenerator(nh, rate, grid_width, grid_height, grid_resolution);
//    ros::Rate r(rate);

//}

//// manually add obstacles
////    Eigen::Vector3d origin_1; origin_1 << 0, 0, 0;
////    Eigen::Vector3d radius_1; radius_1 << 1, 1, 1;

////    Eigen::Vector3d origin_2; origin_2 << 5, 0, 0;
////    Eigen::Vector3d radius_2; radius_2 << 1, 1, 1;

////    obs_gen.add_obstacle_viz(origin_1, radius_1);
////    obs_gen.add_obstacle_viz(origin_2, radius_2);

//void ObstacleGeneratorROS::run()
//{
//    while(ros::ok())
//    {
//        obs_gen.run();
//        ros::spinOnce();
//        r.sleep();
//    }
//}
