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

    auto obs_gen = ObstacleGenerator(rate, 600, 600, 0.01);
//    obs_gen.setMaxObstacleNum(50);
    obs_gen.setBlindAngle(-M_PI/4, M_PI/4);

//    Eigen::Vector3d obstacle_origin(0, 0, 0);
//    Eigen::Vector3d obstacle_radius(0.01, 0.01, 0.01);

    int iteration = 0;
    double radial_increase = 2 * M_PI / 10000;
    ros::Rate r(rate);
    while(ros::ok())
    {

        double min_angle = -M_PI/4 + iteration * radial_increase;
        double max_angle = M_PI/4 + iteration * radial_increase;

//        obs_gen.setBlindAngle(min_angle, max_angle);


        obs_gen.run();
        ros::spinOnce();
        r.sleep();
        iteration++;
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
