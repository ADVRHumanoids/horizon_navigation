# Pipeline
Depending on the sensor used, different processes can be launch: 

## velodyne only

```mon launch horizon_navigation velodyne_mapping.launch```

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
- an occupancy_map `/velodyne_map`: centered at `/base_link`
- an occupancy_map `/blind_zone`: centered at `/base_link`

The first map contains the obstacles data, while the second map is for the visualization of the blind zone. The maps are solidal with the robot motion.


## sonar only

```mon launch horizon_navigation sonar_mapping.launch```

1. the sensor data from one sonar get transformed into grid_map values. If an obstacle is sensed inside the sonar range, an occupancy cone is created at the distance detected.

1. more sensors can be added in the ```sonar_config.yaml``` to include datas of all the desired sensor in the same occupancy map.

Input:
- one or multiple topics specified in the ```sonar_config.yaml```
Output:
- an occupancy_map `/sonar_map`: centered at `/base_link`

## velodyne + sonar simple fusion

```mon launch horizon_navigation fusion_mapping.launch```

The simple fusion of the velodyne and sonar processes in the same grid_map.

Input:
- a frame `/odom` from robot odometry
- one or multiple topics `/cloud_in` from velodynes
- one or multiple topics specified in the ```sonar_config.yaml```

Output:Horizon
- an occupancy_map `/fusion_map`: centered at `/base_link`

## navigation

```mon launch horizon_navigation navigation.launch```

A modifed version of the previous velodyne/sonar fusion.
This version uses the sonar data to clear out "false positive" obstacles inside the blind zone. If some cells inside the sonar range has a occupied value but the sonar does not sense any obstacle, that cell is cleared.

Input:
- a frame `/odom` from robot odometry
- one or multiple topics `/cloud_in` from velodynes
- one or multiple topics specified in the ```sonar_config.yaml```

Output:
- an occupancy_map `/navigation_map`: centered at `/base_link`
- an occupancy_map `/sonar_map`: centered at `/base_link`




