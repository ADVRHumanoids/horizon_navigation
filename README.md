# horizon_navigation

Utilities to integrate perception in the Horizon environment.

For now, data coming from velodynes and sonar are supported. All the utilities are based on the [grid_map](https://github.com/ANYbotics/grid_map) package from ANYbotics. 

The map is based on velodyne, sonar data and robot odometry. 
This utility is created with three criteria in mind:
1. dynamic obstacles
    * objects can disappear or move around the map
1. blind zone in proximity of the sensor
    * centered at robot's `/base_link`.
    * objects entering it are fixed in the map to avoid misdetection.
1. sonars' data validates the presence of the obstacle in the blind zone

The occupancy map occupied cells are transformed into sphere instances that can be digested by [Horizon](https://github.com/ADVRHumanoids/horizon) and thrown in the optimization process for obstacle avoidance.

For more implementation details, check:
 * [horizon obstacles management](README_obstacles.md)
 * [occupancy map generation](README_occupancy_map.md)

---
 ### full example

- ``mon launch concert_gazebo concert.launch  velodyne:=true ultrasound:=true`` (for robot simulation with sensors)
- ``mon launch concert_odometry concert_odometry.launch`` (for robot odometry, publishes frame ``/odom``)
- ``mon launch horizon_navigation navigation.launch`` (generates the occupancy map)
- ``mon launch concert_horizon concert_controller.launch xbot:=true`` (horizon controller)

for visualization in rviz:
- ``roscd horizon_navigation && rviz -d config/rviz/concert_nav.rviz``

