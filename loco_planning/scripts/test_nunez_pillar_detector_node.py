#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy as ros

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from utils.pillar_detector import PillarDetector


class TestNunezPillarDetectorNode:
    def __init__(self):
        ros.init_node("test_nunez_pillar_detector_node", anonymous=False)

        self.scan_topic = ros.get_param("~scan_topic", "/limo0/scan")

        self.sigma_r = ros.get_param("~sigma_r", 0.01)
        self.sigma_phi = ros.get_param("~sigma_phi", -1.0)

        self.dbscan_eps = ros.get_param("~dbscan_eps", 0.12)
        self.dbscan_min_samples = ros.get_param("~dbscan_min_samples", 3)

        self.min_cluster_points = ros.get_param("~min_cluster_points", 5)
        self.max_cluster_points = ros.get_param("~max_cluster_points", 80)

        self.pillar_min_radius = ros.get_param("~pillar_min_radius", 0.10)
        self.pillar_max_radius = ros.get_param("~pillar_max_radius", 0.30)

        self.max_geometric_rmse = ros.get_param("~max_geometric_rmse", 0.05)
        self.circle_fit_max_nfev = ros.get_param("~circle_fit_max_nfev", 100)

        self.max_detections_per_scan = ros.get_param("~max_detections_per_scan", 20)

        self.marker_height = ros.get_param("~marker_height", 0.50)
        self.covariance_sigma_scale = ros.get_param("~covariance_sigma_scale", 2.0)

        self.detector = PillarDetector(
            sigma_r=self.sigma_r,
            sigma_phi=self.sigma_phi,
            dbscan_eps=self.dbscan_eps,
            dbscan_min_samples=self.dbscan_min_samples,
            min_cluster_points=self.min_cluster_points,
            max_cluster_points=self.max_cluster_points,
            pillar_min_radius=self.pillar_min_radius,
            pillar_max_radius=self.pillar_max_radius,
            max_geometric_rmse=self.max_geometric_rmse,
            circle_fit_max_nfev=self.circle_fit_max_nfev,
            max_detections_per_scan=self.max_detections_per_scan
        )

        self.status_pub = ros.Publisher(
            "/pillar_detector_nunez/status",
            Float64MultiArray,
            queue_size=1
        )

        self.detections_laser_pub = ros.Publisher(
            "/pillar_detector_nunez/detections_laser",
            Float64MultiArray,
            queue_size=1
        )

        self.marker_pub = ros.Publisher(
            "/pillar_detector_nunez/markers_laser",
            MarkerArray,
            queue_size=1
        )

        self.scan_sub = ros.Subscriber(
            self.scan_topic,
            LaserScan,
            self.scan_callback,
            queue_size=1
        )

        ros.loginfo("==========================================")
        ros.loginfo("TEST PILLAR DETECTOR NODE")
        ros.loginfo("Output detections are in LaserScan frame.")
        ros.loginfo("scan_topic              = %s", self.scan_topic)
        ros.loginfo("sigma_r                 = %.6f", self.sigma_r)
        ros.loginfo("sigma_phi               = %.6f (-1 means angle_increment/sqrt(12))", self.sigma_phi)
        ros.loginfo("dbscan_eps              = %.3f", self.dbscan_eps)
        ros.loginfo("dbscan_min_samples      = %d", self.dbscan_min_samples)
        ros.loginfo("min_cluster_points      = %d", self.min_cluster_points)
        ros.loginfo("max_cluster_points      = %d", self.max_cluster_points)
        ros.loginfo("circle_fit_max_nfev     = %d", self.circle_fit_max_nfev)
        ros.loginfo("radius limits           = [%.3f, %.3f]", self.pillar_min_radius, self.pillar_max_radius)
        ros.loginfo("max_geometric_rmse      = %.3f", self.max_geometric_rmse)
        ros.loginfo("max_detections_per_scan = %d", self.max_detections_per_scan)
        ros.loginfo("covariance_sigma_scale  = %.2f", self.covariance_sigma_scale)
        ros.loginfo("==========================================")

    def scan_callback(self, scan_msg):
        detections = self.detector.detect(scan_msg)

        self.publish_detections_laser(detections)
        self.publish_status(detections)
        self.publish_markers_laser(detections, scan_msg.header)

    def publish_detections_laser(self, detections):
        msg = Float64MultiArray()
        data = []

        for detection in detections:
            center = detection["center_laser"]
            P = detection["P_center_laser"]

            data.extend([
                center[0],
                center[1],
                P[0, 0],
                P[0, 1],
                P[1, 0],
                P[1, 1],
                detection["radius"],
                detection["geometric_rmse"],
                detection["arc_angle_deg"],
                detection["n_points"]
            ])

        msg.data = data
        self.detections_laser_pub.publish(msg)

    def publish_status(self, detections):
        msg = Float64MultiArray()

        if len(detections) == 0:
            msg.data = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ]

            self.status_pub.publish(msg)
            return

        radii = []
        rmses = []
        pxx_values = []
        pyy_values = []
        pxy_values = []
        arcs = []
        n_points = []

        for detection in detections:
            P = detection["P_center_laser"]

            radii.append(detection["radius"])
            rmses.append(detection["geometric_rmse"])
            pxx_values.append(P[0, 0])
            pyy_values.append(P[1, 1])
            pxy_values.append(P[0, 1])
            arcs.append(detection["arc_angle_deg"])
            n_points.append(detection["n_points"])

        msg.data = [
            len(detections),
            np.mean(radii),
            np.mean(rmses),
            np.mean(pxx_values),
            np.mean(pyy_values),
            np.mean(pxy_values),
            np.mean(arcs),
            np.mean(n_points)
        ]

        self.status_pub.publish(msg)

    def covariance_marker_from_detection(self, detection, scan_header, marker_id):
        center = detection["center_laser"]
        P = detection["P_center_laser"]

        P = 0.5 * (P + P.T)

        try:
            eigenvalues, eigenvectors = np.linalg.eigh(P)
        except np.linalg.LinAlgError:
            return None

        eigenvalues = np.maximum(eigenvalues, 0.0)

        order = np.argsort(eigenvalues)[::-1]

        lambda_major = eigenvalues[order[0]]
        lambda_minor = eigenvalues[order[1]]

        vector_major = eigenvectors[:, order[0]]

        yaw = math.atan2(vector_major[1], vector_major[0])

        axis_major = self.covariance_sigma_scale * math.sqrt(lambda_major)
        axis_minor = self.covariance_sigma_scale * math.sqrt(lambda_minor)

        marker = Marker()
        marker.header.frame_id = scan_header.frame_id
        marker.header.stamp = scan_header.stamp

        marker.ns = "pillar_detector_covariance_laser"
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = self.marker_height + 0.03

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        marker.scale.x = 2.0 * axis_major
        marker.scale.y = 2.0 * axis_minor
        marker.scale.z = 0.02

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.45

        return marker

    def publish_markers_laser(self, detections, scan_header):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 0

        for detection in detections:
            center = detection["center_laser"]
            radius = detection["radius"]

            marker = Marker()
            marker.header.frame_id = scan_header.frame_id
            marker.header.stamp = scan_header.stamp

            marker.ns = "pillar_detector_centers_laser"
            marker.id = marker_id
            marker_id = marker_id + 1

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = self.marker_height / 2.0

            marker.pose.orientation.w = 1.0

            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = self.marker_height

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.65

            marker_array.markers.append(marker)

            covariance_marker = self.covariance_marker_from_detection(
                detection,
                scan_header,
                marker_id
            )

            marker_id = marker_id + 1

            if covariance_marker is not None:
                marker_array.markers.append(covariance_marker)

            text_marker = Marker()
            text_marker.header.frame_id = scan_header.frame_id
            text_marker.header.stamp = scan_header.stamp

            text_marker.ns = "pillar_detector_text_laser"
            text_marker.id = marker_id
            marker_id = marker_id + 1

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = center[0]
            text_marker.pose.position.y = center[1]
            text_marker.pose.position.z = self.marker_height + 0.30
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.16

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = (
                "method=%s xL=%.2f yL=%.2f r=%.3f rmse=%.3f n=%d"
                %
                (
                    detection["method"],
                    center[0],
                    center[1],
                    detection["radius"],
                    detection["geometric_rmse"],
                    detection["n_points"]
                )
            )

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def spin(self):
        ros.spin()


if __name__ == "__main__":
    node = TestNunezPillarDetectorNode()
    node.spin()