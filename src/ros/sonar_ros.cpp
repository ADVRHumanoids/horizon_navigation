#include <ros/sonar_ros.h>

SonarOccupancyMapROS::SonarOccupancyMapROS():
//    _sensor_topic_names(sensor_topic_names),
    _tfBuffer(),
    _tfListener(_tfBuffer),
    _rate(30.0)
{

    _nh = ros::NodeHandle("");
    _nhpr = ros::NodeHandle("~");

    // get sensor info
    auto sensors_config = load_param_file(_nhpr);

    for (SonarConfig sonar_info : sensors_config)
    {
        _sensor_topics[sonar_info.sensor_name] = sonar_info.topic_name;

        auto sonar = std::make_shared<SonarOccupancyMap::Sonar>();
        sonar->arc_resolution = sonar_info.arc_resolution;
        sonar->detection_range = sonar_info.detection_range;

        _sensors[sonar_info.sensor_name] = sonar;
    }

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
        sensor.second->arc_resolution = 30;
        _sonar_occupancy_map->addSensor(sensor.first, sensor.second);
    }
}

std::vector<SonarOccupancyMapROS::SonarConfig> SonarOccupancyMapROS::load_param_file(ros::NodeHandle nh)
{

    std::vector<SonarConfig> sonar_config;

    std::string config_file_path;
    if (!nh.getParam("config_file", config_file_path))
    {
        ROS_ERROR("Failed to get param 'config_file'");
    }

    try
    {
        YAML::Node config = YAML::LoadFile(config_file_path);
        if (config["sensors"])
        {
            for (const auto& sensor : config["sensors"])
            {

                std::string sensor_name = sensor.first.as<std::string>();
                YAML::Node sensor_config_yaml = sensor.second;

                for (auto sensor_info : sonar_config)
                {
                    if (sensor_info.sensor_name == sensor_name)
                    {
                        ROS_ERROR("Multiple sensors added with the same name: %s", sensor_name.c_str());
                    }
                }

                SonarConfig sensor_info;
                sensor_info.sensor_name = sensor_name;

                if (sensor_config_yaml["topic_name"])
                {
                    sensor_info.topic_name = sensor_config_yaml["topic_name"].as<std::string>();
                }
                else
                {
                    ROS_ERROR("Sensor '%s' requires param 'topic_name'.", sensor_name.c_str());
                }

                if (sensor_config_yaml["detection_range"])
                {
                    sensor_info.detection_range = sensor_config_yaml["detection_range"].as<double>();
                }
                else
                {
                    ROS_WARN("Sensor '%s' missing param 'detection_range'. Using default: '%f'", sensor_name.c_str(), sensor_info.detection_range);
                }

                if (sensor_config_yaml["arc_resolution"])
                {
                    sensor_info.arc_resolution = sensor_config_yaml["arc_resolution"].as<double>();
                }
                else
                {
                    ROS_WARN("Sensor '%s' missing param 'arc_resolution'. Using default: '%f'", sensor_name.c_str(), sensor_info.arc_resolution);
                }

                sonar_config.push_back(sensor_info);
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
    }

    return sonar_config;
}

std::map<std::string, sensor_msgs::Range::ConstPtr> SonarOccupancyMapROS::getRanges()
{
    return _range_msgs;
}

SonarOccupancyMap::Ptr SonarOccupancyMapROS::getSonarOccupancyMap()
{
    return _sonar_occupancy_map;
}

void SonarOccupancyMapROS::init_publishers()
{
    _sonar_map_publisher = _nh.advertise<nav_msgs::OccupancyGrid>("sonar_map", 1, true);
}

void SonarOccupancyMapROS::init_subscribers()
{
    for (auto sensor : _sensor_topics)
    {
        _range_subs[sensor.first] = _nh.subscribe<sensor_msgs::Range>(sensor.second, 1000, &SonarOccupancyMapROS::rangeCallback, this);
        auto msg = ros::topic::waitForMessage<sensor_msgs::Range>(sensor.second, _nh);
        _range_msgs[sensor.first] = msg;
    }
}

void SonarOccupancyMapROS::init_transform()
{
    try
    {
        for (auto &sensor : _sensors)
        {
            auto origin_T_sensor_transform = _tfBuffer.lookupTransform("base_link",
                                                                       _range_msgs[sensor.first]->header.frame_id,
                                                                       ros::Time(0), //_latest_local_map->header.stamp
                                                                       ros::Duration(1.0));

            sensor.second->origin_T_sensor = tf2::transformToEigen(origin_T_sensor_transform);
        }


    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
}

void SonarOccupancyMapROS::rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    _range_msgs[msg->header.frame_id] = msg;
}

void SonarOccupancyMapROS::spin()
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

void SonarOccupancyMapROS::update()
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
        }
    }

    _map = _sonar_occupancy_map->getMap();

}

grid_map::GridMap SonarOccupancyMapROS::getMap()
{
    return _map;
}
