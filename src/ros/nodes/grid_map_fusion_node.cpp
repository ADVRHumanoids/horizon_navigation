#include <ros/navigation_ros.h>
#include <ros/sonar_ros.h>

void publishRectangle(ros::Publisher& marker_pub, float center_x, float center_y, float width, float height)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "base_link";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "rectangle";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.01; // Line width

    // Set the color -- be sure to set alpha to something non-zero!
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point p;

    // Calculate vertices
    p.x = center_x - width / 2.0;
    p.y = center_y - height / 2.0;
    line_strip.points.push_back(p);

    p.x = center_x + width / 2.0;
    p.y = center_y - height / 2.0;
    line_strip.points.push_back(p);

    p.x = center_x + width / 2.0;
    p.y = center_y + height / 2.0;
    line_strip.points.push_back(p);

    p.x = center_x - width / 2.0;
    p.y = center_y + height / 2.0;
    line_strip.points.push_back(p);

    p.x = center_x - width / 2.0;
    p.y = center_y - height / 2.0;
    line_strip.points.push_back(p);

    marker_pub.publish(line_strip);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_transformer_node");
    ros::NodeHandle nh("");
    double rate;
    ros::param::param<double>("~rate", rate, 100);
    ROS_INFO("Running at rate: %f", rate);

    auto maptransformer = VelodyneOccupancyMapROS(rate);
    auto sonar = SonarOccupancyMapROS();

    auto map_fusion_pub = nh.advertise<nav_msgs::OccupancyGrid>("/fusion_map", 1);
    auto obstacle_pub = nh.advertise<nav_msgs::OccupancyGrid>("/filtered_local_map", 1);
    auto blind_pub = nh.advertise<nav_msgs::OccupancyGrid>("/blind_zone", 1);
    auto sonar_pub = nh.advertise<nav_msgs::OccupancyGrid>("/sonar_map", 1);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("sonar_zone", 10);


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

                grid_map::Length reset_zone_length(1.2 + 0.6, 0.8 + 0.6);
                grid_map::Index reset_zone_size(reset_zone_length.x() / obstacle_map.getResolution(), reset_zone_length.y() / obstacle_map.getResolution());


                grid_map::Position reset_zone_start_pos = obstacle_map.getPosition() + grid_map::Position(reset_zone_length.x() / 2.0, reset_zone_length.y() / 2.0);
                grid_map::Index reset_zone_start_index;

                obstacle_map.getIndex(reset_zone_start_pos, reset_zone_start_index);


                for (grid_map::SubmapIterator iterator(obstacle_map, reset_zone_start_index, reset_zone_size); !iterator.isPastEnd(); ++iterator)
                {
                    grid_map::Position map_position;
                    obstacle_map.getPosition(*iterator, map_position);
                    grid_map::Index sonar_index;
                    sonar_map.getIndex(map_position, sonar_index);

//                    if (obstacle_map.at("obstacles", *iterator) > 50.0)
//                    {
////                        std::cout << sonar_map.at("sonar_map", sonar_index) << std::endl;
//                        if (sonar_map.at("sonar_map", sonar_index) != 100.0)
//                        {
//                            obstacle_map.at("obstacles", *iterator) = 0.0;
//                        }
//                    }

                    if (sonar_map.at("sonar_map", sonar_index) > 50.0)
                    {
                        obstacle_map.at("obstacles", *iterator) = sonar_map.at("sonar_map", sonar_index);
                    }
                }


                publishRectangle(marker_pub, 0.0, 0.0, reset_zone_length.x(), reset_zone_length.y());

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
