#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from obstacles_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math


class MapToObstacles:
    def __init__(self, step_m):
        rospy.init_node("map_to_polygons")
        #Node subscribes to /map
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        #latch publish only once and keep it alive
        self.obs_pub = rospy.Publisher("/obstacles", ObstacleArrayMsg, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher("/markers/obstacles", MarkerArray, queue_size=1, latch=True)
        self.step_m = step_m

    def map_callback(self, msg):
        # Convert occupancy grid to numpy image
        width, height = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Occupied cells (>=50 considered obstacle)
        occupied = np.uint8((data > 50) * 255)
        # Find contours (obstacles)
        contours, _ = cv2.findContours(occupied, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #debug
        #plt.imshow(occupied, cmap="gray")
        #for cnt in contours:
        #    xs = cnt[:, 0, 0]
        #    ys = cnt[:, 0, 1]
        #    plt.plot(xs, ys, 'r-')
        #plt.xlabel("x[m]")
        #plt.ylabel("y[m]")
        #plt.show(block=False)
        #plt.pause(0.01)

        arr = ObstacleArrayMsg()
        arr.header = msg.header
        markers = MarkerArray()

        for cnt in contours:
            obs = ObstacleMsg()
            #plot all points
            # for pt in cnt:
            #     j = pt[0][0]  # column index
            #     i = pt[0][1]  # row index
            #     # Convert from grid to world coordinates
            #     x = msg.info.origin.position.x + j * msg.info.resolution
            #     y = msg.info.origin.position.y + i * msg.info.resolution
            #     obs.polygon.points.append(Point32(x=x, y=y, z=0))
            #     # Optional: also add a visualization marker for each point
            #     m = Marker()
            #     m.header = msg.header
            #     m.header.frame_id = "map"
            #     m.ns = "obstacle_points"
            #     m.id = len(markers.markers)
            #     m.type = Marker.SPHERE
            #     m.action = Marker.ADD
            #     m.pose.position.x = x
            #     m.pose.position.y = y
            #     m.pose.position.z = 0.05
            #     m.scale.x = 0.05
            #     m.scale.y = 0.05
            #     m.scale.z = 0.05
            #     m.color.a = 1.0
            #     m.color.r = 1.0
            #     m.color.g = 0.0
            #     m.color.b = 0.0
            #     markers.markers.append(m)
            # arr.obstacles.append(obs)

            # plot only some samples discretized
            self.sampled_points = self.discretize_contour(cnt, msg.info.resolution, msg.info.origin, step_m=self.step_m)
            for (x, y) in self.sampled_points:
                point = Point32()
                point.x = x
                point.y = y
                obs.polygon.points.append(point)
                obs.radius = self.step_m
                # Visualization marker (sphere per point)
                m = Marker()
                m.header = msg.header
                m.header.frame_id = "map"
                m.ns = "obstacle_points"
                m.id = len(markers.markers)
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.05
                m.pose.orientation.x = 0
                m.pose.orientation.y = 0
                m.pose.orientation.z =0
                m.pose.orientation.w =1
                m.scale.x = 0.2
                m.scale.y = 0.2
                m.scale.z = 0.2
                m.color.a = 1.0
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                markers.markers.append(m)
            arr.obstacles.append(obs)

        self.obs_pub.publish(arr)
        self.marker_pub.publish(markers)

    def discretize_contour(self, contour, resolution, origin, step_m=0.25):
        """Convert OpenCV contour to downsampled world points"""
        points = []
        last_x, last_y = None, None

        for pt in contour:
            j = pt[0][0]  # column index
            i = pt[0][1]  # row index

            x = origin.position.x + j * resolution
            y = origin.position.y + i * resolution

            if last_x is None:
                points.append((x, y))
                last_x, last_y = x, y
            else:
                dist = math.hypot(x - last_x, y - last_y)
                if dist >= step_m:   # only keep points spaced at least step_m apart
                    points.append((x, y))
                    last_x, last_y = x, y
        return points

if __name__ == "__main__":
    try:
        p = MapToObstacles(step_m=0.2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
