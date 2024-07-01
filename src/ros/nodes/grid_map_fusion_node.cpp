#include <ros/map_transformer_ros.h>
#include <ros/sonar_ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_transformer_node");
    ros::NodeHandle nh("");
    double rate;
    ros::param::param<double>("~rate", rate, 100);
    ROS_INFO("Running at rate: %f", rate);

    auto maptransformer = MapTransformerROS(rate);
    auto sonar = SonarROS();

    auto map_fusion_pub = nh.advertise<nav_msgs::OccupancyGrid>("/fusion_map", 1);
    auto blind_pub = nh.advertise<nav_msgs::OccupancyGrid>("/blind_zone", 1);

    ros::Rate r(rate);


    while (ros::ok())
    {
        try
        {

            auto success = maptransformer.update();
            sonar.update();

            if (success)
            {
                auto obstacle_map = maptransformer.getMap();
                auto sonar_map = sonar.getMap();
                auto blind_zone = maptransformer.getBlindZone();

//                grid_map::GridMap fusion_map;

//                fusion_map.add("cazzi", obstacle_map.get("obstacles") + sonar_map.get("sonar_map"));

//                obstacle_map.add("obstacles", sonar_map.get("sonar_map"));
//                sonar_map.get("sonar_map") += obstacle_map.get("obstacles");
                obstacle_map.addDataFrom(sonar_map, false, true, false, {"sonar_map"});


                nav_msgs::OccupancyGrid obstacle_message;
                grid_map::GridMapRosConverter::toOccupancyGrid(obstacle_map, "obstacles", -100, 100, obstacle_message);
                map_fusion_pub.publish(obstacle_message);

                nav_msgs::OccupancyGrid blind_zone_message;
                grid_map::GridMapRosConverter::toOccupancyGrid(blind_zone, "obstacles", -100, 100, blind_zone_message);
                blind_pub.publish(blind_zone_message);


            }

            r.sleep();
            ros::spinOnce();

        }
        catch (ros::Exception &ex)
        {
            ROS_WARN("%s", ex.what());
        }

    }
    maptransformer.update();

    return 0;
}
