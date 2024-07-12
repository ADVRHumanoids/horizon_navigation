# Pipeline

The ```ObstacleGenerator``` generates horizon obstacles from one or multiple [occupancy maps](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html).

The obstacles and the robots are represented as spheres and ellipsoid respectively.

Given the `obstacles radius`, the `origin w.r.t. the robot`, and the `robot ellipsoid radii`, a repulsion cost is defined between the obstacles and the robot body.

### details

1. only the N cells closest to the robot are considered obstacles. The closest cells are computed in a circular fashion: for each discretized angle around the robot, only the closest obstacle is considered.

2. a blind cone ignores all the obstacles in front of the robot

