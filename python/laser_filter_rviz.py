#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
import yaml

class LaserFilterRviz:

    def __init__(self, config_file):

        rospy.init_node('rectangle_marker_publisher')
        self.marker_pub = rospy.Publisher('filtered_robot_footprint', Marker, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz

        self.config_file = config_file
        self.get_params_from_config()

    def get_params_from_config(self):

        with open(self.config_file) as f:
            params = yaml.safe_load(f)

        self.frame_id = params['scan_filter_chain'][0]['params']['box_frame']

        self.min_x = params['scan_filter_chain'][0]['params']['min_x']
        self.max_x = params['scan_filter_chain'][0]['params']['max_x']
        self.min_y = params['scan_filter_chain'][0]['params']['min_y']
        self.max_y = params['scan_filter_chain'][0]['params']['max_y']
        self.min_z = params['scan_filter_chain'][0]['params']['min_z']
        self.max_z = params['scan_filter_chain'][0]['params']['max_z']

    def create_rectangle_marker(self):

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "footprint_robot"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        points = [
            Point(self.min_x, self.max_y, 0),
            Point(self.max_x, self.max_y, 0),
            Point(self.max_x, self.min_y, 0),
            Point(self.min_x, self.min_y, 0),
            Point(self.min_x, self.max_y, 0)  # Close the loop
        ]

        marker.points = points
        return marker

    def run(self):

        while not rospy.is_shutdown():
            marker = self.create_rectangle_marker()
            self.marker_pub.publish(marker)
            self.rate.sleep()

if __name__ == "__main__":


    if len(sys.argv) < 2:
        rospy.logerr("Usage: rectangle_marker.py <config_file>")
        sys.exit(1)

    config_file = sys.argv[1]

    lfrv = LaserFilterRviz(config_file)
    lfrv.run()
