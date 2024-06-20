# horizon_navigation
Utilities to integrate perception in the Horizon environment.

For now, data coming from velodynes are supported.

`mapping.launch` launches a cluster of ros nodes to generate an occupancy map centered at the robot's `/base_link`. 
The map is based on velodyne data and a robot odometry. This utility is created with two criteria in mind:
1. dynamic obstacles
    * objects can disappear or move around the map
2. blind zone in proximity of the sensor
    * centered at `/base_link`.
    * objects entering it are fixed in the map to avoid misdetection.

## Pipeline
The architecture is built as following:
1. the velodyne data get transformed in a laserscan.
1. the laserscan is used to create:
    - a local map using [costmap2d](https://wiki.ros.org/costmap_2d)
    - a world frame `/map` using [gmapping](https://wiki.ros.org/gmapping)
1. *the blind-zone local map* is created by filtering out data within a central region 
1. the world frame `/map` is used to create a world map that gets updated by the *blind-zone local map*
1. an output data is generated taking the filtered input of the global map

This allows to update the obstacles outside the blind zone while "remembering" the obstacles that enters the blind zone using the odometry (given the hypotesis that the velodyne has a proximal blind zone)

Input:
- a frame `/odom` from robot odometry
- one or multiple topics `/cloud_in` from velodynes

Output:
- an occupancy_map `/filtered_world_map`: centered at `/map`
- an occupancy_map `/filtered_local_map`: centered at `/base_link`

The two occupancy map contains the same data, the only difference is that one is global while the other is solidal with the robot motion.




