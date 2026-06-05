#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
from sklearn.cluster import DBSCAN

from utils.FilterAndPolar2xy_LidarData import filterAndPolar2XYWithCovariance
from utils.CircleFitting import fitCircleNunezFull
from utils.CircleFitting import computeCircleArcAngle


class PillarDetector:
    def __init__(
        self,
        sigma_r=0.01,
        sigma_phi=-1.0,
        dbscan_eps=0.12,
        dbscan_min_samples=3,
        min_cluster_points=5,
        max_cluster_points=80,
        pillar_min_radius=0.10,
        pillar_max_radius=0.30,
        max_geometric_rmse=0.05,
        circle_fit_max_nfev=100,
        max_detections_per_scan=20
    ):
        self.sigma_r = sigma_r
        self.sigma_phi = sigma_phi

        self.dbscan_eps = dbscan_eps
        self.dbscan_min_samples = dbscan_min_samples

        self.min_cluster_points = min_cluster_points
        self.max_cluster_points = max_cluster_points

        self.pillar_min_radius = pillar_min_radius
        self.pillar_max_radius = pillar_max_radius

        self.max_geometric_rmse = max_geometric_rmse
        self.circle_fit_max_nfev = circle_fit_max_nfev

        self.max_detections_per_scan = max_detections_per_scan

    def detect(self, scan_msg):
        points, covariances, ranges, angles = filterAndPolar2XYWithCovariance(
            scan_msg,
            self.sigma_r,
            self.sigma_phi
        )

        if points.shape[0] == 0:
            return []

        clusters = self.cluster_points(points)

        detections = []

        for cluster_indices in clusters:
            detection = self.fit_cluster(
                points,
                covariances,
                cluster_indices
            )

            if detection is None:
                continue

            detections.append(detection)

        if self.max_detections_per_scan > 0:
            if len(detections) > self.max_detections_per_scan:
                detections = detections[0:self.max_detections_per_scan]

        return detections

    def cluster_points(self, points):
        if points.shape[0] < self.min_cluster_points:
            return []

        dbscan = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_samples
        )

        labels = dbscan.fit_predict(points)

        clusters = []

        unique_labels = sorted(set(labels))

        for label in unique_labels:
            if label < 0:
                continue

            cluster_indices = np.where(labels == label)[0]

            if cluster_indices.shape[0] < self.min_cluster_points:
                continue

            if cluster_indices.shape[0] > self.max_cluster_points:
                continue

            clusters.append(cluster_indices)

        return clusters

    def fit_cluster(self, points, covariances, cluster_indices):
        cluster_points = points[cluster_indices, :]
        cluster_covariances = covariances[cluster_indices, :, :]

        result = fitCircleNunezFull(
            cluster_points,
            cluster_covariances,
            max_nfev=self.circle_fit_max_nfev
        )

        if result is None:
            return None

        center = result["center"]
        radius = result["radius"]
        rmse = result["geometric_rmse"]
        P_center = result["P_center"]

        if radius < self.pillar_min_radius:
            return None

        if radius > self.pillar_max_radius:
            return None

        if rmse > self.max_geometric_rmse:
            return None

        if not np.all(np.isfinite(center)):
            return None

        if not np.all(np.isfinite(P_center)):
            return None

        P_center = 0.5 * (P_center + P_center.T)

        if P_center[0, 0] < 0.0:
            return None

        if P_center[1, 1] < 0.0:
            return None

        arc_angle_deg = computeCircleArcAngle(
            cluster_points,
            center
        )

        detection = {
            "center_laser": center,
            "P_center_laser": P_center,
            "P_circle": result["P_circle"],
            "radius": radius,
            "geometric_rmse": rmse,
            "arc_angle_deg": arc_angle_deg,
            "n_points": cluster_points.shape[0],
            "method": "nunez",
            "s2": result["s2"],
            "residual_norm": result["residual_norm"],
            "dof": result["dof"]
        }

        return detection