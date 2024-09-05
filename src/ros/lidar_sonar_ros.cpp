#include <ros/lidar_sonar_ros.h>

LidarSonarROS::LidarSonarROS(double rate):
    _rate(rate)
{

    _nh = ros::NodeHandle("");
    _nhpr = ros::NodeHandle("~");

    _sonar_ros = std::make_unique<SonarOccupancyMapROS>();
    // _velodyne_ros = std::make_unique<VelodyneOccupancyMapROS>(rate);

    _local_sonar_map_publisher = _nh.advertise<nav_msgs::OccupancyGrid>("sonar_map", 1, true);
    // _velodyne_map_publisher = _nh.advertise<nav_msgs::OccupancyGrid>("velodyne_map", 1, true);


}

bool LidarSonarROS::update()
{

    // _latest_local_map = _velodyne_ros->getSensedOccupancyMap();
    _local_sonar_grid_map = _sonar_ros->getMap();

    return true;

}

void LidarSonarROS::spin()
{
    while (_nh.ok())
    {
        try
        {
            auto success = update();

            if (success)
            {
                nav_msgs::OccupancyGrid sonar_occupancy_grid_map;
                grid_map::GridMapRosConverter::toOccupancyGrid(_local_sonar_grid_map, "sonar", -100, 100, sonar_occupancy_grid_map);
                _local_sonar_map_publisher.publish(sonar_occupancy_grid_map);

            //    _velodyne_map_publisher.publish(_latest_local_map);

            }

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        _rate.sleep();
        ros::spinOnce();
    }
}
