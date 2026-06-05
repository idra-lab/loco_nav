#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import datetime
import numpy as np

import rospy as ros
import tf
import message_filters

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from utils.math_tools import wrapToPi

from utils.pillar_detector import PillarDetector

from utils.ukf_utils import ukf_prediction
from utils.ukf_utils import ukf_landmark_measurement_distribution
from utils.ukf_utils import ukf_landmark_measurement_update
from utils.ukf_utils import augment_state_with_landmark
from utils.ukf_utils import landmark_position_base2map

from utils.data_association_utils import associate_detections_to_landmarks
from utils.data_association_utils import build_measurement_debug_from_associations
from utils.data_association_utils import find_nearest_landmark_euclidean
from utils.data_association_utils import get_landmark_position


class TestLandmarkSlamUkfNode:

    def __init__(self):
        ros.init_node("test_landmark_slam_ukf_node", anonymous=False)

        self.scan_topic = ros.get_param("~scan_topic", "/limo0/scan")
        self.odom_topic = ros.get_param("~odom_topic", "/limo0/odom")

        self.map_frame = ros.get_param("~map_frame", "map")
        self.odom_frame = ros.get_param("~odom_frame", "limo0/odom")
        self.base_frame = ros.get_param("~base_frame", "limo0/base_link")
        self.publish_map_to_odom = ros.get_param("~publish_map_to_odom", True)
        self.tf_timeout = ros.get_param("~tf_timeout", 0.10)

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

        self.initial_pose_std_x = ros.get_param("~initial_pose_std_x", 0.05)
        self.initial_pose_std_y = ros.get_param("~initial_pose_std_y", 0.05)
        self.initial_pose_std_yaw = ros.get_param("~initial_pose_std_yaw", 0.05)

        self.odom_process_var_x = ros.get_param("~odom_process_var_x", 0.0001)
        self.odom_process_var_y = ros.get_param("~odom_process_var_y", 0.0001)
        self.odom_process_var_yaw = ros.get_param("~odom_process_var_yaw", 0.01)

        self.mahalanobis_gate = ros.get_param("~mahalanobis_gate", 5.991)
        self.landmark_init_distance = ros.get_param("~landmark_init_distance", 0.60)
        self.max_landmarks = ros.get_param("~max_landmarks", 20)

        self.ukf_alpha = ros.get_param("~ukf_alpha", 0.30)
        self.ukf_beta = ros.get_param("~ukf_beta", 2.0)
        self.ukf_k = ros.get_param("~ukf_k", 0.0)

        self.marker_height = ros.get_param("~marker_height", 0.50)
        self.robot_marker_diameter = ros.get_param("~robot_marker_diameter", 0.22)
        self.covariance_marker_sigma = ros.get_param("~covariance_marker_sigma", 2.0)

        self.enable_logging = ros.get_param("~enable_logging", True)
        self.log_dir = ros.get_param("~log_dir", "/root/slam_logs")

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

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.is_initialized = False
        self.X = None
        self.P = None
        self.last_odom_pose = None
        self.last_map_odom_pose = None

        self.landmarks = []
        self.next_landmark_id = 0

        self.last_update_count = 0
        self.last_mean_update_innovation_norm = 0.0
        self.last_mean_update_mahalanobis = 0.0

        self.P_odom_delta = np.diag([
            self.odom_process_var_x,
            self.odom_process_var_y,
            self.odom_process_var_yaw
        ])

        self.robot_history = []
        self.landmark_history = []
        self.measurement_history = []
        self.association_history = []
        self.map_odom_history = []

        self.run_dir = None

        if self.enable_logging:
            self.create_log_directory()

        self.status_pub = ros.Publisher(
            "/test_landmark_slam_ukf/status",
            Float64MultiArray,
            queue_size=1
        )
        self.robot_state_pub = ros.Publisher(
            "/test_landmark_slam_ukf/robot_state",
            Float64MultiArray,
            queue_size=1
        )
        self.augmented_state_pub = ros.Publisher(
            "/test_landmark_slam_ukf/augmented_state",
            Float64MultiArray,
            queue_size=1
        )
        self.detections_base_pub = ros.Publisher(
            "/test_landmark_slam_ukf/detections_base",
            Float64MultiArray,
            queue_size=1
        )
        self.landmarks_pub = ros.Publisher(
            "/test_landmark_slam_ukf/landmarks",
            Float64MultiArray,
            queue_size=1
        )
        self.measurement_debug_pub = ros.Publisher(
            "/test_landmark_slam_ukf/measurement_debug",
            Float64MultiArray,
            queue_size=1
        )
        self.association_debug_pub = ros.Publisher(
            "/test_landmark_slam_ukf/association_debug",
            Float64MultiArray,
            queue_size=1
        )
        self.covariance_debug_pub = ros.Publisher(
            "/test_landmark_slam_ukf/covariance_debug",
            Float64MultiArray,
            queue_size=1
        )
        self.marker_pub = ros.Publisher(
            "/test_landmark_slam_ukf/markers",
            MarkerArray,
            queue_size=1
        )

        self.scan_sub = message_filters.Subscriber(self.scan_topic, LaserScan)
        self.odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.odom_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.synced_callback)

        ros.on_shutdown(self.on_shutdown)

        ros.loginfo("==========================================")
        ros.loginfo("TEST LANDMARK SLAM UKF NODE - MAP->ODOM")
        ros.loginfo("scan_topic              = %s", self.scan_topic)
        ros.loginfo("odom_topic              = %s", self.odom_topic)
        ros.loginfo("map_frame               = %s", self.map_frame)
        ros.loginfo("odom_frame              = %s", self.odom_frame)
        ros.loginfo("base_frame              = %s", self.base_frame)
        ros.loginfo("publish_map_to_odom     = %s", str(self.publish_map_to_odom))
        ros.loginfo("sigma_r                 = %.6f", self.sigma_r)
        ros.loginfo("sigma_phi               = %.6f", self.sigma_phi)
        ros.loginfo("dbscan_eps              = %.3f", self.dbscan_eps)
        ros.loginfo("dbscan_min_samples      = %d", self.dbscan_min_samples)
        ros.loginfo(
            "radius limits           = [%.3f, %.3f]",
            self.pillar_min_radius,
            self.pillar_max_radius
        )
        ros.loginfo("max_geometric_rmse      = %.3f", self.max_geometric_rmse)
        ros.loginfo(
            "P_odom_delta diag       = [%.6f, %.6f, %.6f]",
            self.odom_process_var_x,
            self.odom_process_var_y,
            self.odom_process_var_yaw
        )
        ros.loginfo("mahalanobis_gate        = %.6f", self.mahalanobis_gate)
        ros.loginfo("ukf_alpha               = %.6f", self.ukf_alpha)
        ros.loginfo("ukf_beta                = %.6f", self.ukf_beta)
        ros.loginfo("ukf_k                   = %.6f", self.ukf_k)
        ros.loginfo("landmark_init_distance  = %.6f", self.landmark_init_distance)
        ros.loginfo("max_landmarks           = %d", self.max_landmarks)
        ros.loginfo("enable_logging          = %s", str(self.enable_logging))
        if self.run_dir is not None:
            ros.loginfo("log run_dir             = %s", self.run_dir)
        ros.loginfo("==========================================")

    def synced_callback(self, scan_msg, odom_msg):
        odom_pose = self.get_odom_pose(odom_msg)

        if not self.is_initialized:
            self.initialize_robot_state(odom_pose)

            if self.publish_map_to_odom:
                self.publish_map_to_odom_transform(odom_pose, scan_msg.header.stamp)

            return

        self.X, self.P = ukf_prediction(
            self.X,
            self.P,
            self.P_odom_delta,
            self.ukf_alpha,
            self.ukf_beta,
            self.ukf_k,
            self.last_odom_pose,
            odom_pose
        )

        self.last_odom_pose = odom_pose.copy()

        detections_laser = self.detector.detect(scan_msg)
        detections_base = self.transform_detections_to_base(
            detections_laser,
            scan_msg.header.frame_id
        )

        associations, unassigned_detection_indices, association_debug = associate_detections_to_landmarks(
            self.X,
            self.P,
            detections_base,
            self.landmarks,
            ukf_landmark_measurement_distribution,
            self.mahalanobis_gate,
            self.ukf_alpha,
            self.ukf_beta,
            self.ukf_k
        )

        self.perform_ukf_updates(detections_base, associations)

        self.initialize_unassigned_detections(
            detections_base,
            unassigned_detection_indices
        )

        measurement_debug = build_measurement_debug_from_associations(
            self.X,
            self.P,
            detections_base,
            self.landmarks,
            associations,
            ukf_landmark_measurement_distribution,
            self.ukf_alpha,
            self.ukf_beta,
            self.ukf_k
        )

        self.publish_detections_base(detections_base)
        self.publish_robot_state(odom_pose)
        self.publish_augmented_state()
        self.publish_landmarks()
        self.publish_measurement_debug(measurement_debug)
        self.publish_association_debug(association_debug)
        self.publish_covariance_debug()

        self.publish_status(
            detections_base,
            odom_pose,
            measurement_debug,
            associations,
            unassigned_detection_indices
        )

        if self.publish_map_to_odom:
            self.publish_map_to_odom_transform(odom_pose, scan_msg.header.stamp)

        self.publish_markers(detections_base, measurement_debug, scan_msg.header.stamp)

        if self.enable_logging:
            self.log_current_step(
                scan_msg.header.stamp,
                odom_pose,
                detections_base,
                measurement_debug,
                associations,
                unassigned_detection_indices
            )

    def get_odom_pose(self, odom_msg):
        pose = odom_msg.pose.pose
        q = pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        return np.array([pose.position.x, pose.position.y, yaw])

    def pose2d_to_matrix(self, pose):
        x = pose[0]
        y = pose[1]
        theta = pose[2]

        c = math.cos(theta)
        s = math.sin(theta)

        T = np.eye(3)
        T[0, 0] = c
        T[0, 1] = -s
        T[1, 0] = s
        T[1, 1] = c
        T[0, 2] = x
        T[1, 2] = y

        return T

    def matrix_to_pose2d(self, T):
        x = T[0, 2]
        y = T[1, 2]
        theta = math.atan2(T[1, 0], T[0, 0])

        return np.array([x, y, theta])

    def publish_map_to_odom_transform(self, odom_pose, stamp):
        T_map_base = self.pose2d_to_matrix(self.X[0:3])
        T_odom_base = self.pose2d_to_matrix(odom_pose)

        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        map_odom_pose = self.matrix_to_pose2d(T_map_odom)
        self.last_map_odom_pose = map_odom_pose.copy()

        q = tf.transformations.quaternion_from_euler(
            0.0,
            0.0,
            map_odom_pose[2]
        )

        self.tf_broadcaster.sendTransform(
            (map_odom_pose[0], map_odom_pose[1], 0.0),
            q,
            stamp,
            self.odom_frame,
            self.map_frame
        )

    def initialize_robot_state(self, odom_pose):
        self.X = np.array([
            odom_pose[0],
            odom_pose[1],
            odom_pose[2]
        ])

        self.P = np.diag([
            self.initial_pose_std_x * self.initial_pose_std_x,
            self.initial_pose_std_y * self.initial_pose_std_y,
            self.initial_pose_std_yaw * self.initial_pose_std_yaw
        ])

        self.last_odom_pose = odom_pose.copy()
        self.is_initialized = True

        ros.loginfo(
            "Initialized robot state from odom: x=%.3f y=%.3f yaw=%.3f",
            self.X[0],
            self.X[1],
            self.X[2]
        )

    def transform_detections_to_base(self, detections_laser, source_frame):
        detections_base = []

        for detection in detections_laser:
            detection_base = self.transform_detection_to_base(detection, source_frame)

            if detection_base is not None:
                detections_base.append(detection_base)

        return detections_base

    def transform_detection_to_base(self, detection, source_frame):
        try:
            self.tf_listener.waitForTransform(
                self.base_frame,
                source_frame,
                ros.Time(0),
                ros.Duration(self.tf_timeout)
            )

            translation, quaternion = self.tf_listener.lookupTransform(
                self.base_frame,
                source_frame,
                ros.Time(0)
            )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            ros.logwarn_throttle(
                2.0,
                "Waiting for TF %s <- %s",
                self.base_frame,
                source_frame
            )
            return None

        T = tf.transformations.quaternion_matrix(quaternion)
        R = T[0:2, 0:2]
        t = np.array([translation[0], translation[1]])

        z_laser = detection["center_laser"]
        P_laser = detection["P_center_laser"]

        z_base = R @ z_laser + t
        P_base = R @ P_laser @ R.T
        P_base = 0.5 * (P_base + P_base.T)

        detection_base = dict(detection)
        detection_base["center_base"] = z_base
        detection_base["P_center_base"] = P_base
        detection_base["source_frame"] = source_frame
        detection_base["base_frame"] = self.base_frame

        return detection_base

    def perform_ukf_updates(self, detections_base, associations):
        update_innovation_norms = []
        update_mahalanobis_values = []

        for association in associations:
            detection_index = association["detection_index"]
            landmark_index = association["landmark_list_index"]

            detection = detections_base[detection_index]
            landmark = self.landmarks[landmark_index]

            z_obs = detection["center_base"]
            R_measurement = detection["P_center_base"]
            landmark_state_index = landmark["state_index"]

            self.X, self.P, update_info = ukf_landmark_measurement_update(
                self.X,
                self.P,
                z_obs,
                R_measurement,
                landmark_state_index,
                self.ukf_alpha,
                self.ukf_beta,
                self.ukf_k
            )

            if update_info is None:
                continue

            landmark["n_observations"] = landmark["n_observations"] + 1

            update_innovation_norms.append(update_info["innovation_norm"])
            update_mahalanobis_values.append(association["mahalanobis_squared"])

        self.last_update_count = len(update_innovation_norms)

        if len(update_innovation_norms) == 0:
            self.last_mean_update_innovation_norm = 0.0
        else:
            self.last_mean_update_innovation_norm = float(np.mean(update_innovation_norms))

        if len(update_mahalanobis_values) == 0:
            self.last_mean_update_mahalanobis = 0.0
        else:
            self.last_mean_update_mahalanobis = float(np.mean(update_mahalanobis_values))

    def initialize_unassigned_detections(self, detections_base, unassigned_detection_indices):
        for detection_index in unassigned_detection_indices:
            if len(self.landmarks) >= self.max_landmarks:
                continue

            detection = detections_base[detection_index]

            z_base = detection["center_base"]
            R_measurement = detection["P_center_base"]

            landmark_position = landmark_position_base2map(self.X, z_base)

            nearby_landmark = find_nearest_landmark_euclidean(
                self.X,
                self.landmarks,
                landmark_position,
                self.landmark_init_distance
            )

            if nearby_landmark is not None:
                nearby_landmark["n_observations"] = nearby_landmark["n_observations"] + 1
                continue

            self.X, self.P, landmark_state_index = augment_state_with_landmark(
                self.X,
                self.P,
                z_base,
                R_measurement,
                self.ukf_alpha,
                self.ukf_beta,
                self.ukf_k
            )

            landmark = {
                "id": self.next_landmark_id,
                "state_index": landmark_state_index,
                "n_observations": 1
            }

            self.landmarks.append(landmark)
            self.next_landmark_id = self.next_landmark_id + 1

            position = get_landmark_position(self.X, landmark)

            ros.loginfo(
                "Added landmark id=%d at state_index=%d x=%.3f y=%.3f",
                landmark["id"],
                landmark["state_index"],
                position[0],
                position[1]
            )

    def compute_relative_landmark_covariance_map(self, landmark):
        idx = landmark["state_index"]

        P_rr = self.P[0:2, 0:2]
        P_ll = self.P[idx:idx + 2, idx:idx + 2]
        P_rl = self.P[0:2, idx:idx + 2]
        P_lr = self.P[idx:idx + 2, 0:2]

        P_rel = P_ll + P_rr - P_lr - P_rl
        P_rel = 0.5 * (P_rel + P_rel.T)

        return P_rel

    def publish_detections_base(self, detections_base):
        msg = Float64MultiArray()
        data = []

        for detection in detections_base:
            center = detection["center_base"]
            P = detection["P_center_base"]

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
        self.detections_base_pub.publish(msg)

    def publish_robot_state(self, odom_pose):
        msg = Float64MultiArray()

        msg.data = [
            1.0,
            self.X[0],
            self.X[1],
            self.X[2],
            self.P[0, 0],
            self.P[1, 1],
            self.P[2, 2],
            odom_pose[0],
            odom_pose[1],
            odom_pose[2],
            self.X.shape[0],
            len(self.landmarks)
        ]

        self.robot_state_pub.publish(msg)

    def publish_augmented_state(self):
        msg = Float64MultiArray()
        msg.data = list(self.X)
        self.augmented_state_pub.publish(msg)

    def publish_landmarks(self):
        msg = Float64MultiArray()
        data = []

        for landmark in self.landmarks:
            idx = landmark["state_index"]
            position = get_landmark_position(self.X, landmark)

            data.extend([
                landmark["id"],
                idx,
                position[0],
                position[1],
                landmark["n_observations"],
                self.P[idx, idx],
                self.P[idx, idx + 1],
                self.P[idx + 1, idx],
                self.P[idx + 1, idx + 1]
            ])

        msg.data = data
        self.landmarks_pub.publish(msg)

    def publish_measurement_debug(self, measurement_debug):
        msg = Float64MultiArray()
        data = []

        for item in measurement_debug:
            z_obs = item["z_obs"]
            z_pred = item["z_pred"]
            innovation = item["innovation"]

            data.extend([
                item["detection_index"],
                item["landmark_id"],
                item["state_index"],
                z_obs[0],
                z_obs[1],
                z_pred[0],
                z_pred[1],
                innovation[0],
                innovation[1],
                item["innovation_norm"],
                item["mahalanobis_squared"],
                item["radius"],
                item["geometric_rmse"],
                item["n_points"]
            ])

        msg.data = data
        self.measurement_debug_pub.publish(msg)

    def publish_association_debug(self, association_debug):
        msg = Float64MultiArray()
        data = []

        for item in association_debug:
            data.extend([
                item["detection_index"],
                item["landmark_id"],
                item["state_index"],
                item["mahalanobis_squared"],
                item["innovation_norm"],
                item["accepted"]
            ])

        msg.data = data
        self.association_debug_pub.publish(msg)

    def publish_covariance_debug(self):
        msg = Float64MultiArray()
        data = []

        robot_cov = self.P[0:3, 0:3]

        data.extend([
            self.P[0, 0],
            self.P[1, 1],
            self.P[2, 2],
            self.P[0, 1],
            self.P[0, 2],
            self.P[1, 2],
            float(np.trace(robot_cov))
        ])

        for landmark in self.landmarks:
            idx = landmark["state_index"]
            P_ll = self.P[idx:idx + 2, idx:idx + 2]
            P_rel = self.compute_relative_landmark_covariance_map(landmark)

            data.extend([
                landmark["id"],
                idx,
                P_ll[0, 0],
                P_ll[0, 1],
                P_ll[1, 0],
                P_ll[1, 1],
                P_rel[0, 0],
                P_rel[0, 1],
                P_rel[1, 0],
                P_rel[1, 1],
                float(np.trace(P_rel)),
                float(np.linalg.det(P_rel))
            ])

        msg.data = data
        self.covariance_debug_pub.publish(msg)

    def publish_status(
            self,
            detections_base,
            odom_pose,
            measurement_debug,
            associations,
            unassigned_detection_indices
    ):
        radii = []
        rmses = []
        measurement_pxx = []
        measurement_pyy = []
        innovation_norms = []
        mahalanobis_values = []

        for detection in detections_base:
            P = detection["P_center_base"]
            radii.append(detection["radius"])
            rmses.append(detection["geometric_rmse"])
            measurement_pxx.append(P[0, 0])
            measurement_pyy.append(P[1, 1])

        for item in measurement_debug:
            innovation_norms.append(item["innovation_norm"])
            mahalanobis_values.append(item["mahalanobis_squared"])

        mean_radius = float(np.mean(radii)) if len(radii) > 0 else 0.0
        mean_rmse = float(np.mean(rmses)) if len(rmses) > 0 else 0.0
        mean_measurement_pxx = float(np.mean(measurement_pxx)) if len(measurement_pxx) > 0 else 0.0
        mean_measurement_pyy = float(np.mean(measurement_pyy)) if len(measurement_pyy) > 0 else 0.0
        mean_innovation_norm = float(np.mean(innovation_norms)) if len(innovation_norms) > 0 else 0.0
        mean_mahalanobis = float(np.mean(mahalanobis_values)) if len(mahalanobis_values) > 0 else 0.0

        msg = Float64MultiArray()

        msg.data = [
            len(detections_base),
            odom_pose[0],
            odom_pose[1],
            odom_pose[2],
            self.X[0],
            self.X[1],
            self.X[2],
            mean_radius,
            mean_rmse,
            mean_measurement_pxx,
            mean_measurement_pyy,
            self.P[0, 0],
            self.P[1, 1],
            self.P[2, 2],
            len(self.landmarks),
            self.X.shape[0],
            len(associations),
            len(unassigned_detection_indices),
            mean_innovation_norm,
            mean_mahalanobis,
            self.last_update_count,
            self.last_mean_update_innovation_norm,
            self.last_mean_update_mahalanobis
        ]

        self.status_pub.publish(msg)



    # Markers section

    def publish_markers(self, detections_base, measurement_debug, stamp):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 0

        robot_marker = self.create_robot_marker(stamp, marker_id)
        marker_id = marker_id + 1
        marker_array.markers.append(robot_marker)

        heading_marker = self.create_robot_heading_marker(stamp, marker_id)
        marker_id = marker_id + 1
        marker_array.markers.append(heading_marker)

        for detection in detections_base:
            detection_ukf_marker = self.create_detection_ukf_map_marker(
                detection,
                stamp,
                marker_id
            )
            marker_id = marker_id + 1
            marker_array.markers.append(detection_ukf_marker)

        for item in measurement_debug:
            predicted_marker = self.create_predicted_measurement_marker(
                item,
                stamp,
                marker_id
            )
            marker_id = marker_id + 1
            marker_array.markers.append(predicted_marker)

        for landmark in self.landmarks:
            landmark_marker = self.create_landmark_marker(
                landmark,
                stamp,
                marker_id
            )
            marker_id = marker_id + 1
            marker_array.markers.append(landmark_marker)

            landmark_relative_cov_marker = self.create_landmark_relative_covariance_marker(
                landmark,
                stamp,
                marker_id
            )
            marker_id = marker_id + 1

            if landmark_relative_cov_marker is not None:
                marker_array.markers.append(landmark_relative_cov_marker)

            text_marker = self.create_landmark_text_marker(
                landmark,
                stamp,
                marker_id
            )
            marker_id = marker_id + 1
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def create_robot_marker(self, stamp, marker_id):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "estimated_robot_body"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = self.X[0]
        marker.pose.position.y = self.X[1]
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.robot_marker_diameter
        marker.scale.y = self.robot_marker_diameter
        marker.scale.z = 0.30

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def create_robot_heading_marker(self, stamp, marker_id):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "estimated_robot_heading"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        yaw = self.X[2]

        marker.pose.position.x = self.X[0]
        marker.pose.position.y = self.X[1]
        marker.pose.position.z = 0.35
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        marker.scale.x = 0.35
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker


    def create_detection_ukf_map_marker(self, detection, stamp, marker_id):
        z_base = detection["center_base"]
        radius = detection["radius"]

        position_map_ukf = landmark_position_base2map(self.X, z_base)

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "detections_map_using_ukf_pose"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = position_map_ukf[0]
        marker.pose.position.y = position_map_ukf[1]
        marker.pose.position.z = self.marker_height / 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = self.marker_height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.85

        return marker

    def create_predicted_measurement_marker(self, item, stamp, marker_id):
        z_pred = item["z_pred"]

        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = stamp
        marker.ns = "predicted_measurements_base"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = z_pred[0]
        marker.pose.position.y = z_pred[1]
        marker.pose.position.z = self.marker_height + 0.15
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.85

        return marker

    def create_landmark_marker(self, landmark, stamp, marker_id):
        position = get_landmark_position(self.X, landmark)

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "augmented_state_landmarks"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = self.marker_height / 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.40
        marker.scale.y = 0.40
        marker.scale.z = self.marker_height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.70

        return marker


    def create_landmark_relative_covariance_marker(self, landmark, stamp, marker_id):
        position = get_landmark_position(self.X, landmark)
        P_rel = self.compute_relative_landmark_covariance_map(landmark)

        return self.create_covariance_ellipse_marker(
            position,
            P_rel,
            self.map_frame,
            "landmark_covariance_relative",
            stamp,
            marker_id,
            0.0,
            1.0,
            1.0,
            0.95
        )

    def create_covariance_ellipse_marker(
            self,
            center,
            covariance_2d,
            frame_id,
            namespace,
            stamp,
            marker_id,
            r,
            g,
            b,
            a
    ):
        covariance_2d = 0.5 * (covariance_2d + covariance_2d.T)

        try:
            eigenvalues, eigenvectors = np.linalg.eigh(covariance_2d)
        except np.linalg.LinAlgError:
            return None

        if eigenvalues[0] < 0.0 or eigenvalues[1] < 0.0:
            return None

        axis_0 = self.covariance_marker_sigma * math.sqrt(eigenvalues[1])
        axis_1 = self.covariance_marker_sigma * math.sqrt(eigenvalues[0])

        angle = math.atan2(eigenvectors[1, 1], eigenvectors[0, 1])

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.025

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        c = math.cos(angle)
        s = math.sin(angle)

        for i in range(0, 73):
            t = 2.0 * math.pi * float(i) / 72.0
            ex = axis_0 * math.cos(t)
            ey = axis_1 * math.sin(t)

            px = center[0] + c * ex - s * ey
            py = center[1] + s * ex + c * ey

            point = Point()
            point.x = px
            point.y = py
            point.z = 0.04

            marker.points.append(point)

        return marker

    def create_landmark_text_marker(self, landmark, stamp, marker_id):
        position = get_landmark_position(self.X, landmark)

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "pillar_index"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = self.marker_height + 0.35
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.28

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.text = "idx = %d" % landmark["id"]

        return marker

   # Log section

    def create_log_directory(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.log_dir, "run_" + timestamp)

        if not os.path.exists(self.run_dir):
            os.makedirs(self.run_dir)

    def stamp_to_sec(self, stamp):
        if stamp is None:
            return ros.Time.now().to_sec()

        t = stamp.to_sec()

        if t <= 0.0:
            return ros.Time.now().to_sec()

        return t

    def log_current_step(
            self,
            stamp,
            odom_pose,
            detections_base,
            measurement_debug,
            associations,
            unassigned_detection_indices
    ):
        t = self.stamp_to_sec(stamp)

        map_odom_x = 0.0
        map_odom_y = 0.0
        map_odom_theta = 0.0

        if self.last_map_odom_pose is not None:
            map_odom_x = self.last_map_odom_pose[0]
            map_odom_y = self.last_map_odom_pose[1]
            map_odom_theta = self.last_map_odom_pose[2]

        self.robot_history.append({
            "t": t,
            "x_ukf": self.X[0],
            "y_ukf": self.X[1],
            "theta_ukf": self.X[2],
            "Pxx": self.P[0, 0],
            "Pyy": self.P[1, 1],
            "Ptheta": self.P[2, 2],
            "sigma_x": math.sqrt(max(self.P[0, 0], 0.0)),
            "sigma_y": math.sqrt(max(self.P[1, 1], 0.0)),
            "sigma_theta": math.sqrt(max(self.P[2, 2], 0.0)),
            "odom_x": odom_pose[0],
            "odom_y": odom_pose[1],
            "odom_theta": odom_pose[2],
            "map_odom_x": map_odom_x,
            "map_odom_y": map_odom_y,
            "map_odom_theta": map_odom_theta,
            "state_dim": self.X.shape[0],
            "n_landmarks": len(self.landmarks)
        })

        self.map_odom_history.append({
            "t": t,
            "map_odom_x": map_odom_x,
            "map_odom_y": map_odom_y,
            "map_odom_theta": map_odom_theta
        })

        for landmark in self.landmarks:
            idx = landmark["state_index"]
            position = get_landmark_position(self.X, landmark)
            P_ll = self.P[idx:idx + 2, idx:idx + 2]
            P_rel = self.compute_relative_landmark_covariance_map(landmark)

            self.landmark_history.append({
                "t": t,
                "landmark_id": landmark["id"],
                "state_index": idx,
                "x": position[0],
                "y": position[1],
                "n_observations": landmark["n_observations"],
                "Pxx": P_ll[0, 0],
                "Pxy": P_ll[0, 1],
                "Pyx": P_ll[1, 0],
                "Pyy": P_ll[1, 1],
                "trace": float(np.trace(P_ll)),
                "det": float(np.linalg.det(P_ll)),
                "Pxx_relative": P_rel[0, 0],
                "Pxy_relative": P_rel[0, 1],
                "Pyx_relative": P_rel[1, 0],
                "Pyy_relative": P_rel[1, 1],
                "trace_relative": float(np.trace(P_rel)),
                "det_relative": float(np.linalg.det(P_rel))
            })

        for item in measurement_debug:
            z_obs = item["z_obs"]
            z_pred = item["z_pred"]
            innovation = item["innovation"]

            self.measurement_history.append({
                "t": t,
                "detection_index": item["detection_index"],
                "landmark_id": item["landmark_id"],
                "state_index": item["state_index"],
                "z_obs_x": z_obs[0],
                "z_obs_y": z_obs[1],
                "z_pred_x": z_pred[0],
                "z_pred_y": z_pred[1],
                "innovation_x": innovation[0],
                "innovation_y": innovation[1],
                "innovation_norm": item["innovation_norm"],
                "mahalanobis_squared": item["mahalanobis_squared"],
                "radius": item["radius"],
                "geometric_rmse": item["geometric_rmse"],
                "n_points": item["n_points"]
            })

        innovation_norms = []
        mahalanobis_values = []

        for item in measurement_debug:
            innovation_norms.append(item["innovation_norm"])
            mahalanobis_values.append(item["mahalanobis_squared"])

        if len(innovation_norms) > 0:
            mean_innovation_norm = float(np.mean(innovation_norms))
            max_innovation_norm = float(np.max(innovation_norms))
        else:
            mean_innovation_norm = 0.0
            max_innovation_norm = 0.0

        if len(mahalanobis_values) > 0:
            mean_mahalanobis = float(np.mean(mahalanobis_values))
            max_mahalanobis = float(np.max(mahalanobis_values))
        else:
            mean_mahalanobis = 0.0
            max_mahalanobis = 0.0

        self.association_history.append({
            "t": t,
            "n_detections": len(detections_base),
            "n_associations": len(associations),
            "n_unassigned": len(unassigned_detection_indices),
            "n_landmarks": len(self.landmarks),
            "mean_innovation_norm": mean_innovation_norm,
            "max_innovation_norm": max_innovation_norm,
            "mean_mahalanobis_squared": mean_mahalanobis,
            "max_mahalanobis_squared": max_mahalanobis,
            "last_update_count": self.last_update_count,
            "last_mean_update_innovation_norm": self.last_mean_update_innovation_norm,
            "last_mean_update_mahalanobis": self.last_mean_update_mahalanobis
        })

    def write_csv(self, filename, rows):
        if self.run_dir is None:
            return

        path = os.path.join(self.run_dir, filename)

        if len(rows) == 0:
            with open(path, "w") as f:
                f.write("")
            return

        fieldnames = list(rows[0].keys())

        with open(path, "w") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            for row in rows:
                writer.writerow(row)

    def save_all_csv(self):
        if not self.enable_logging:
            return

        if self.run_dir is None:
            return

        self.write_csv("robot_history.csv", self.robot_history)
        self.write_csv("landmark_history.csv", self.landmark_history)
        self.write_csv("measurement_history.csv", self.measurement_history)
        self.write_csv("association_history.csv", self.association_history)
        self.write_csv("map_odom_history.csv", self.map_odom_history)

    def get_robot_array(self):
        if len(self.robot_history) == 0:
            return None

        keys = list(self.robot_history[0].keys())
        data = {}

        for key in keys:
            data[key] = np.array([row[key] for row in self.robot_history], dtype=float)

        return data

    def get_association_array(self):
        if len(self.association_history) == 0:
            return None

        keys = list(self.association_history[0].keys())
        data = {}

        for key in keys:
            data[key] = np.array([row[key] for row in self.association_history], dtype=float)

        return data

    def write_summary(self):
        if self.run_dir is None:
            return

        robot_data = self.get_robot_array()
        association_data = self.get_association_array()

        summary_path = os.path.join(self.run_dir, "summary.txt")

        with open(summary_path, "w") as f:
            f.write("SLAM RELATIVE LOCALIZATION SUMMARY\n")
            f.write("===================\n\n")

            f.write("Samples robot: %d\n" % len(self.robot_history))
            f.write("Samples landmarks: %d\n" % len(self.landmark_history))
            f.write("Samples measurements: %d\n" % len(self.measurement_history))
            f.write("Samples associations: %d\n\n" % len(self.association_history))

            if robot_data is not None:
                f.write("Final robot state UKF:\n")
                f.write("  x     = %.6f\n" % robot_data["x_ukf"][-1])
                f.write("  y     = %.6f\n" % robot_data["y_ukf"][-1])
                f.write("  theta = %.6f rad\n\n" % robot_data["theta_ukf"][-1])

                f.write("Final robot covariance:\n")
                f.write("  Pxx     = %.9f\n" % robot_data["Pxx"][-1])
                f.write("  Pyy     = %.9f\n" % robot_data["Pyy"][-1])
                f.write("  Ptheta  = %.9f\n" % robot_data["Ptheta"][-1])
                f.write("  sigma_x     = %.6f m\n" % robot_data["sigma_x"][-1])
                f.write("  sigma_y     = %.6f m\n" % robot_data["sigma_y"][-1])
                f.write("  sigma_theta = %.6f rad\n\n" % robot_data["sigma_theta"][-1])

            if association_data is not None:
                f.write("Association/update summary:\n")
                f.write(
                    "  mean detections      = %.6f\n"
                    % float(np.mean(association_data["n_detections"]))
                )
                f.write(
                    "  mean associations    = %.6f\n"
                    % float(np.mean(association_data["n_associations"]))
                )
                f.write(
                    "  mean innovation norm = %.6f m\n"
                    % float(np.mean(association_data["mean_innovation_norm"]))
                )
                f.write(
                    "  mean Mahalanobis     = %.6f\n\n"
                    % float(np.mean(association_data["mean_mahalanobis_squared"]))
                )

            f.write("Final landmarks:\n")

            if len(self.landmark_history) > 0:
                latest_by_id = {}

                for row in self.landmark_history:
                    latest_by_id[int(row["landmark_id"])] = row

                for landmark_id in sorted(latest_by_id.keys()):
                    row = latest_by_id[landmark_id]
                    f.write(
                        "  id=%d x=%.6f y=%.6f n_obs=%d trace=%.9f trace_rel=%.9f\n"
                        % (
                            landmark_id,
                            row["x"],
                            row["y"],
                            int(row["n_observations"]),
                            row["trace"],
                            row["trace_relative"]
                        )
                    )

    def on_shutdown(self):
        if not self.enable_logging:
            return

        try:
            self.save_all_csv()
            self.write_summary()

            if self.run_dir is not None:
                ros.loginfo("SLAM logs saved in: %s", self.run_dir)

        except Exception as exc:
            ros.logwarn("Failed to save SLAM logs: %s", str(exc))

    def spin(self):
        ros.spin()


if __name__ == "__main__":
    node = TestLandmarkSlamUkfNode()
    node.spin()