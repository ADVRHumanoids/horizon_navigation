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
    auto obstacle_pub = nh.advertise<nav_msgs::OccupancyGrid>("/filtered_local_map", 1);
    auto blind_pub = nh.advertise<nav_msgs::OccupancyGrid>("/blind_zone", 1);
    auto sonar_pub = nh.advertise<nav_msgs::OccupancyGrid>("/sonar_map", 1);

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


                auto small_map = obstacle_map.getSubmap(sonar_map.getPosition(), blind_zone.getLength(), success);
//                grid_map::GridMap fusion_map;
//                fusion_map.setFrameId("base_link");
//                fusion_map.setPosition(grid_map::Position(0.0, 0.0));
//                fusion_map.setGeometry(obstacle_map.getLength(), obstacle_map.getResolution());

//                fusion_map.add("cazzi", obstacle_map.get("obstacles") + sonar_map.get("sonar_map"));

//                obstacle_map.add("obstacles", sonar_map.get("sonar_map"));
//                obstacle_map.get("obstacles") += sonar_map.get("sonar_map");
//                obstacle_map.addDataFrom(sonar_map, false, false, false, {"sonar_map"});
//                obstacle_map.addDataFrom(sonar_map, false, false, false, {"sonar_map"});

//                for (auto layer : small_map.getLayers())
//                {
//                    std::cout << layer << std::endl;
//                }

//                obstacle_map.getSubmap(obstacle_map.getPosition(), blind_zone.getLength(), success).get("obstacles") += small_map.get("obstacles");


//                nav_msgs::OccupancyGrid fusion_message;
//                grid_map::GridMapRosConverter::toOccupancyGrid(fusion_map, "cazzi", -100, 100, fusion_message);
//                map_fusion_pub.publish(fusion_message);

                nav_msgs::OccupancyGrid obstacle_message;
                grid_map::GridMapRosConverter::toOccupancyGrid(obstacle_map, "obstacles", -100, 100, obstacle_message);
                obstacle_pub.publish(obstacle_message);

                nav_msgs::OccupancyGrid sonar_message;
                grid_map::GridMapRosConverter::toOccupancyGrid(sonar_map, "sonar_map", -100, 100, sonar_message);
                sonar_pub.publish(sonar_message);

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
