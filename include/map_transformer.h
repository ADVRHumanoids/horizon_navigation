#ifndef MAP_TRANSFORMER_H
#define MAP_TRANSFORMER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

class MapTransformer
{
public:

    MapTransformer(double transformed_width = 2.0, double transformed_height = 2.0);
    void update();

private:

    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    nav_msgs::OccupancyGrid additiveUpdate(nav_msgs::OccupancyGrid input_map,
                                           const nav_msgs::OccupancyGrid& updating_map,
                                           const geometry_msgs::TransformStamped& transformStamped,
                                           double patch_width,
                                           double patch_height);
    nav_msgs::OccupancyGrid subtractiveUpdate(nav_msgs::OccupancyGrid input_map,
                                              const nav_msgs::OccupancyGrid& updating_map,
                                              const geometry_msgs::TransformStamped& transformStamped,
                                              double patch_width,
                                              double patch_height);
//    nav_msgs::OccupancyGrid localOverGlobal(nav_msgs::OccupancyGrid input_map,
//                                              const nav_msgs::OccupancyGrid& updating_map,
//                                              const geometry_msgs::TransformStamped& transformStamped,
//                                              double patch_width,
//                                              double patch_height);

    nav_msgs::OccupancyGrid filterLocal(const nav_msgs::OccupancyGrid& global_map,
                                            const nav_msgs::OccupancyGrid& local_map,
                                            const geometry_msgs::TransformStamped& transformStamped,
                                            double patch_width,
                                            double patch_height);

    nav_msgs::OccupancyGrid bFilterA(const nav_msgs::OccupancyGrid& input_map,
                                               const nav_msgs::OccupancyGrid& filter_map,
                                               double patch_width,
                                               double patch_height);

    nav_msgs::OccupancyGrid bOverA(const nav_msgs::OccupancyGrid& map_a,
                                   const nav_msgs::OccupancyGrid& map_b,
                                   const geometry_msgs::TransformStamped& transformStamped,
                                   double patch_width,
                                   double patch_height);

    nav_msgs::OccupancyGrid bToA(nav_msgs::OccupancyGrid a_map,
                                 const nav_msgs::OccupancyGrid& b_map,
                                 const geometry_msgs::TransformStamped& transformStamped,
                                 double patch_width,
                                 double patch_height);



    YAML::Node _config;

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _global_map_sub, _local_map_sub;
    ros::Publisher _transformed_map_pub, _global_map_filtered_pub;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    nav_msgs::OccupancyGrid::ConstPtr _latest_global_map, _latest_local_map;
    nav_msgs::OccupancyGrid     _transformed_map, _world_map; // _map_filter, _global_map_temp, _global_map_filtered;
    double _transformed_width;  // Width of the transformed map in meters
    double _transformed_height; // Height of the transformed map in meters

};

#endif // MAP_TRANSFORMER_H
