#!/usr/bin/env python3

import os
import sys
import cv2
import math
import yaml
import rospy
import rospkg
import numpy as np

from geometry_msgs.msg import Polygon, Point32
from obstacles_msgs.msg import ObstacleArrayMsg
from termcolor import colored

class CreateMap:
    def __init__(self):
        self.padding = 5
        self.resolution = 0.005
        self.borders_done = False
        self.obstacles_done = False

        self.pkg_dir = rospkg.RosPack().get_path('map_pkg')
        self.maps_dir = os.path.join(self.pkg_dir, 'maps')

        rospy.loginfo("Create map node started!")
        rospy.Subscriber("map_borders", Polygon, self.listen_borders, queue_size=1)

    def __create_map(self, msg):
        self.max_x = math.ceil(max(p.x for p in msg.points))
        self.min_x = math.floor(min(p.x for p in msg.points))
        self.max_y = math.ceil(max(p.y for p in msg.points))
        self.min_y = math.floor(min(p.y for p in msg.points))

        height = int((self.max_x - self.min_x + self.padding) / self.resolution)
        width  = int((self.max_y - self.min_y + self.padding) / self.resolution)

        self.img = np.zeros((height, width), dtype=np.uint8)

        pts = np.array([self.__comp_point_wrt_map(p) for p in msg.points], dtype=np.int32)
        cv2.polylines(self.img, [pts], isClosed=True, color=255, thickness=20)

    def __comp_point_wrt_map(self, point):
        return [
            int((self.max_y - point.y + self.padding / 2) / self.resolution),
            int((self.max_x - point.x + self.padding / 2) / self.resolution)
        ]

    def listen_borders(self, msg):
        if self.borders_done:
            return

        rospy.loginfo("Receiving borders...")
        self.borders_done = True

        if len(msg.points) not in [4, 6]:
            raise Exception(f"Invalid number of points ({len(msg.points)}) for the map borders")

        rospy.loginfo(f"Creating map with {len(msg.points)} points...")
        self.__create_map(msg)

        cv2.imwrite(os.path.join(self.maps_dir, "dynamic_map.png"), self.img)
        cv2.imwrite(os.path.join(self.maps_dir, "dynamic_map.pgm"), self.img)

        rospy.loginfo("Initial map borders drawn. Waiting for obstacles...")

        rospy.Subscriber("obstacles", ObstacleArrayMsg, self.listen_obstacles, queue_size=1)

    def listen_obstacles(self, msg):
        if self.obstacles_done:
            return

        rospy.loginfo("Receiving obstacles...")
        self.obstacles_done = True

        for obs in msg.obstacles:
            if len(obs.polygon.points) == 1:
                # Cylinder obstacle
                radius = obs.radius
                center = obs.polygon.points[0]
                pts_tmp = [[
                    center.x + radius * np.cos(theta),
                    center.y + radius * np.sin(theta)
                ] for theta in np.linspace(0, 2 * np.pi, 360)]
                pts = np.array([self.__comp_point_wrt_map(Point32(x=pt[0], y=pt[1])) for pt in pts_tmp], dtype=np.int32)
            else:
                pts = np.array([self.__comp_point_wrt_map(p) for p in obs.polygon.points], dtype=np.int32)

            cv2.fillPoly(self.img, [pts], 255)

        self.img = np.rot90(self.img, k=-1)
        cv2.imwrite(os.path.join(self.maps_dir, "dynamic_map.png"), self.img)
        cv2.imwrite(os.path.join(self.maps_dir, "dynamic_map.pgm"), self.img)

        yaml_path = os.path.join(self.maps_dir, "dynamic_map.yaml")
        origin_x = -((self.max_x - self.min_x + self.padding) / 2)
        origin_y = -((self.max_y - self.min_y + self.padding) / 2)

        yaml_data = {
            'image': 'dynamic_map.pgm',
            'resolution': self.resolution,
            'origin': [origin_x, origin_y, 0.0],
            'negate': 1,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        with open(yaml_path, "w") as f:
            yaml.dump(yaml_data, f)
        print(colored(f"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", "red"))
        rospy.loginfo(f"Map and YAML saved to: {self.maps_dir}")
        rospy.signal_shutdown("Map creation complete.")


def main():
    rospy.init_node('create_map_pgm')

    # Optional: delete old files
    pkg_dir = rospkg.RosPack().get_path('map_pkg')
    maps_dir = os.path.join(pkg_dir, 'maps')
    for fname in ['dynamic_map.png', 'dynamic_map.pgm', 'dynamic_map.yaml']:
        fpath = os.path.join(maps_dir, fname)
        if os.path.exists(fpath):
            os.remove(fpath)

    CreateMap()
    rospy.spin()

if __name__ == "__main__":
    main()
