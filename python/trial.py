from horizon_navigation.pyObstacleGenerator import ObstacleGenerator
from horizon_navigation.pyObstacle import CasadiObstacle
import rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

roscpp_init('node_name', [])
rospy.init_node('obstacle_trial')

rate = 1000
obs_gen = ObstacleGenerator(rate, 60, 60, 0.1)

rate = rospy.Rate(rate)

while True:

    obs_gen.run()
    obs_vec = obs_gen.getObstacles()

    print(obs_vec)

    rate.sleep()
