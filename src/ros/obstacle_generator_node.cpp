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

    auto obs_gen = ObstacleGenerator(600, 600, 0.01);
    obs_gen.setMaxObstacleNum(60);
    obs_gen.setAngleThreshold(0.2);
    obs_gen.setBlindAngle(-M_PI/4, M_PI/4);
    Eigen::Vector3d obstacle_origin(0, 0, 0);
    Eigen::Vector3d obstacle_radius(0.01, 0.01, 0.01);
    int iteration = 0;
    double radial_increase = 2 * M_PI / 10000;
    ros::Rate r(rate);

    while(ros::ok())
    {

//        double min_angle = -M_PI/4 + iteration * radial_increase;
//        double max_angle = M_PI/4 + iteration * radial_increase;

//        obs_gen.setBlindAngle(min_angle, max_angle);


        obs_gen.clearObstacles();

//        // fake obstacles
//        for (double angle=0.0; angle< 2 * M_PI;)
//        {

//            int num_obstacles_row = 3;
//            if (angle == 0.0 || angle == 0.2)
//            {
//                num_obstacles_row = 50;
//            }

//            for (auto i=0; i<num_obstacles_row; i++)
//            {


//                Eigen::Vector3d origin(1.5 * cos(angle), 1.5 * sin(angle), 0);

//                if (angle == 0.0 || angle == 0.2)
//                {
//                    origin << 0.8 * cos(angle), 0.8 * sin(angle), 0;
//                }

//                obstacle_origin[0] = origin[0] + i * 0.03 * cos(angle);
//                obstacle_origin[1] = origin[1] + i * 0.03 * sin(angle);
//                auto obs = std::make_shared<SphereObstacle>(obstacle_origin, obstacle_radius);
//                obs_gen.addObstacle(obs);
//            }

//            angle=angle+0.05;
//        }



        obs_gen._update();
        ros::spinOnce();
        r.sleep();
        iteration++;
    }

    return 0;
}
