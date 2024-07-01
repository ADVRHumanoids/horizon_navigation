#include <ros/sonar_ros.h>

SonarROS::SonarROS():
//    _sensor_topic_names(sensor_topic_names),
    _tfBuffer(),
    _tfListener(_tfBuffer),
    _rate(30.0)
{

    _nh = ros::NodeHandle("");
    _nhpr = ros::NodeHandle("~");

    // get sensor info
    load_param_file();

    ROS_INFO_STREAM("Sensors: ");
    for (auto sensor : _sensor_topics)
    {
        ROS_INFO_STREAM(" --> " + sensor.first);
        ROS_INFO_STREAM("    topic: " + sensor.second);
    }

    // open sensor_map topic
    init_publishers();
    // subscribe to range msgs
    init_subscribers();
    // get transform of sensor from tf2
    init_transform();

    // initialize map
    Eigen::Vector2d map_origin(0.0, 0.0);
    _sonar_occupancy_map = std::make_unique<SonarOccupancyMap>(map_origin);

    // add sensors
    for (auto &sensor : _sensors)
    {
        sensor.second.arc_resolution = 30;
        _sonar_occupancy_map->addSensor(sensor.first, sensor.second);
    }
}

bool SonarROS::load_param_file()
{

    std::string config_file_path;
    if (!_nhpr.getParam("config_file", config_file_path))
    {
        ROS_ERROR("Failed to get param 'config_file'");
        return false;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(config_file_path);
        if (config["sensors"])
        {
            for (const auto& sensor : config["sensors"])
            {
                std::string sensor_name = sensor.first.as<std::string>();
                YAML::Node sensor_config = sensor.second;

                if (_sensors.count(sensor_name) != 0)
                {
                    ROS_ERROR("Multiple sensors added with the same name: %s", sensor_name.c_str());
                }

                if (sensor_config["topic_name"])
                {
                    _sensor_topics[sensor_name] = sensor_config["topic_name"].as<std::string>();
                }
                else
                {
                    ROS_ERROR("Sensor '%s' requires param 'topic_name'.", sensor_name.c_str());
                }

                if (sensor_config["detection_range"])
                {
                    _sensors[sensor_name].detection_range = sensor_config["detection_range"].as<double>();
                }
//                else
//                {
//                    ROS_WARN("Sensor '%s' requires param 'topic_name'.", sensor_name.c_str());
//                }

            }
        }
        else
        {
            ROS_ERROR("No 'sensors' section in config file.");
        }
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR("YAML Exception: %s", e.what());
        return false;
    }

    return true;
}

void SonarROS::init_publishers()
{
    _sonar_map_publisher = _nh.advertise<nav_msgs::OccupancyGrid>("sonar_map", 1, true);
}

void SonarROS::init_subscribers()
{
    for (auto sensor : _sensor_topics)
    {
        _range_subs[sensor.first] = _nh.subscribe<sensor_msgs::Range>(sensor.second, 1000, &SonarROS::rangeCallback, this);
        auto msg = ros::topic::waitForMessage<sensor_msgs::Range>(sensor.second, _nh);
        _range_msgs[sensor.first] = msg;
    }
}

void SonarROS::init_transform()
{
    try
    {
        for (auto &sensor : _sensors)
        {
            auto origin_T_sensor_transform = _tfBuffer.lookupTransform("base_link",
                                                                       _range_msgs[sensor.first]->header.frame_id,
                                                                       ros::Time(0), //_latest_local_map->header.stamp
                                                                       ros::Duration(1.0));

            sensor.second.origin_T_sensor = tf2::transformToEigen(origin_T_sensor_transform);
        }


    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
}

void SonarROS::rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    _range_msgs[msg->header.frame_id] = msg;
}

void SonarROS::spin()
{

    while (_nh.ok())
    {
        try
        {
            update();

            nav_msgs::OccupancyGrid message;
            grid_map::GridMapRosConverter::toOccupancyGrid(_map, "sonar_map", -100, 100, message);
            _sonar_map_publisher.publish(message);

            _rate.sleep();
            ros::spinOnce();

        }
        catch (ros::Exception &ex)
        {
            ROS_WARN("%s", ex.what());
        }

    }
}

void SonarROS::update()
{

    for (auto range_msg : _range_msgs)
    {
        if (range_msg.second)
        {
            _sonar_occupancy_map->setData(range_msg.first,
                                          range_msg.second->header.frame_id,
                                          range_msg.second->range,
                                          range_msg.second->min_range,
                                          range_msg.second->max_range,
                                          range_msg.second->field_of_view);

            _sonar_occupancy_map->update();

            _map = _sonar_occupancy_map->getMap();
        }
    }

}

grid_map::GridMap SonarROS::getMap()
{
    return _map;
}
