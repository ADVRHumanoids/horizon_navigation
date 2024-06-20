#include <ros/sonar_node.h>

SonarRos::SonarRos(std::vector<std::string> sensor_topic_names):
    _sensor_topic_names(sensor_topic_names),
    _tfBuffer(),
    _tfListener(_tfBuffer),
    _rate(30.0)
{

    init_publishers();
    init_subscribers();
    init_transform();

    Eigen::Vector2d map_origin(0.0, 0.0);
    _sonar_occupancy_map = std::make_unique<SonarOccupancyMap>(map_origin, _origin_T_sensor_dict);

}


void SonarRos::init_publishers()
{
    _sonar_map_publisher = _nh.advertise<nav_msgs::OccupancyGrid>("sonar_map", 1, true);
}

void SonarRos::init_subscribers()
{
    for (auto sensor_topic_name : _sensor_topic_names)
    {
        _range_subs[sensor_topic_name] = _nh.subscribe<sensor_msgs::Range>(sensor_topic_name, 1000, &SonarRos::rangeCallback, this);
        auto msg = ros::topic::waitForMessage<sensor_msgs::Range>(sensor_topic_name, _nh);
        _range_msgs[msg->header.frame_id] = msg;
    }


}

void SonarRos::init_transform()
{
    try
    {
        for (auto range_msg : _range_msgs)
        {
            auto origin_T_sensor_transform = _tfBuffer.lookupTransform("base_link",
                                                                       range_msg.second->header.frame_id,
                                                                       ros::Time(0), //_latest_local_map->header.stamp
                                                                       ros::Duration(1.0));

            _origin_T_sensor_dict[range_msg.second->header.frame_id] = tf2::transformToEigen(origin_T_sensor_transform);
        }


    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
}

void SonarRos::rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    _range_msgs[msg->header.frame_id] = msg;
}

void SonarRos::spin()
{

    while (_nh.ok())
    {
        try
        {
            for (auto range_msg : _range_msgs)
            {
                if (range_msg.second)
                {
                    // add data to grid map.
                    _sonar_occupancy_map->setData(range_msg.second->header.frame_id,
                                                  range_msg.second->range,
                                                  range_msg.second->min_range,
                                                  range_msg.second->max_range,
                                                  range_msg.second->field_of_view);

                    _sonar_occupancy_map->update();

                    auto map = _sonar_occupancy_map->getMap();

                    nav_msgs::OccupancyGrid message;
                    GridMapRosConverter::toOccupancyGrid(map, "sonar_map", -100, 100, message);
                    _sonar_map_publisher.publish(message);
                }
            }

        }
        catch (ros::Exception &ex)
        {
            ROS_WARN("%s", ex.what());
        }


        _rate.sleep();
        ros::spinOnce();

    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "grid_map_simple_demo");

    std::vector<std::string> sonar_names;
    sonar_names.push_back("/bosch_uss5/ultrasound_fl_sag");

    auto sr = SonarRos(sonar_names);
    sr.spin();
    return 0;

}
