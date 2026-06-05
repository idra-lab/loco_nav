#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from scipy.optimize import linear_sum_assignment


def compute_mahalanobis_squared(innovation, S):
    try:
        solution = np.linalg.solve(S, innovation)
    except np.linalg.LinAlgError:
        return None

    mahalanobis_squared = float(innovation.T @ solution)

    return mahalanobis_squared


def compute_pair_compatibility(z_obs, z_pred, S):
    innovation = z_obs - z_pred

    mahalanobis_squared = compute_mahalanobis_squared(
        innovation,
        S
    )

    if mahalanobis_squared is None:
        return None

    innovation_norm = float(np.linalg.norm(innovation))

    return {
        "z_obs": z_obs,
        "z_pred": z_pred,
        "innovation": innovation,
        "innovation_norm": innovation_norm,
        "mahalanobis_squared": mahalanobis_squared,
        "S": S
    }


def get_detection_measurement(detection):
    z_obs = detection["center_base"]
    R_measurement = detection["P_center_base"]

    return z_obs, R_measurement


def build_association_cost_matrix(
    X,
    P,
    detections_base,
    landmarks,
    measurement_distribution_function,
    alpha,
    beta,
    k
):
    n_detections = len(detections_base)
    n_landmarks = len(landmarks)

    cost_matrix = np.zeros((n_detections, n_landmarks))
    pair_data = []

    for detection_index in range(n_detections):
        detection = detections_base[detection_index]
        z_obs, R_measurement = get_detection_measurement(detection)

        row_data = []

        for landmark_index in range(n_landmarks):
            landmark = landmarks[landmark_index]
            landmark_state_index = landmark["state_index"]

            distribution = measurement_distribution_function(
                X,
                P,
                R_measurement,
                landmark_state_index,
                alpha,
                beta,
                k
            )

            if distribution is None:
                z_pred = np.array([np.nan, np.nan])
                S = np.full((2, 2), np.nan)
                innovation = np.array([np.nan, np.nan])
                mahalanobis_squared = 1.0e12
                innovation_norm = 1.0e6
            else:
                z_pred = distribution["z_pred"]
                S = distribution["S"]

                compatibility = compute_pair_compatibility(
                    z_obs,
                    z_pred,
                    S
                )

                if compatibility is None:
                    innovation = np.array([np.nan, np.nan])
                    mahalanobis_squared = 1.0e12
                    innovation_norm = 1.0e6
                else:
                    innovation = compatibility["innovation"]
                    mahalanobis_squared = compatibility["mahalanobis_squared"]
                    innovation_norm = compatibility["innovation_norm"]

            cost_matrix[detection_index, landmark_index] = mahalanobis_squared

            row_data.append({
                "detection_index": detection_index,
                "landmark_list_index": landmark_index,
                "landmark_id": landmark["id"],
                "state_index": landmark["state_index"],
                "z_obs": z_obs,
                "z_pred": z_pred,
                "innovation": innovation,
                "innovation_norm": innovation_norm,
                "mahalanobis_squared": mahalanobis_squared,
                "S": S,
                "accepted": 0.0
            })

        pair_data.append(row_data)

    return cost_matrix, pair_data


def build_gated_cost_matrix(cost_matrix, mahalanobis_gate):
    gated_cost_matrix = cost_matrix.copy()

    large_cost = 1.0e12

    invalid_indices = ~np.isfinite(gated_cost_matrix)
    gated_cost_matrix[invalid_indices] = large_cost

    gated_cost_matrix[gated_cost_matrix > mahalanobis_gate] = large_cost

    return gated_cost_matrix


def solve_global_assignment(cost_matrix):
    row_indices, column_indices = linear_sum_assignment(cost_matrix)

    return row_indices, column_indices


def select_gated_associations(
    row_indices,
    column_indices,
    cost_matrix,
    pair_data,
    landmarks,
    mahalanobis_gate
):
    associations = []
    assigned_detection_indices = set()

    for assignment_index in range(len(row_indices)):
        detection_index = row_indices[assignment_index]
        landmark_index = column_indices[assignment_index]

        mahalanobis_squared = cost_matrix[detection_index, landmark_index]
        pair_item = pair_data[detection_index][landmark_index]

        if mahalanobis_squared <= mahalanobis_gate:
            landmark = landmarks[landmark_index]

            association = {
                "detection_index": detection_index,
                "landmark_list_index": landmark_index,
                "landmark_id": landmark["id"],
                "state_index": landmark["state_index"],
                "mahalanobis_squared": mahalanobis_squared,
                "innovation_norm": pair_item["innovation_norm"]
            }

            associations.append(association)
            assigned_detection_indices.add(detection_index)

            pair_item["accepted"] = 1.0

    association_debug = []

    for row_data in pair_data:
        for item in row_data:
            association_debug.append(item)

    return associations, assigned_detection_indices, association_debug


def compute_unassigned_detection_indices(
    n_detections,
    assigned_detection_indices
):
    unassigned_detection_indices = []

    for detection_index in range(n_detections):
        if detection_index not in assigned_detection_indices:
            unassigned_detection_indices.append(detection_index)

    return unassigned_detection_indices


def associate_detections_to_landmarks(
    X,
    P,
    detections_base,
    landmarks,
    measurement_distribution_function,
    mahalanobis_gate,
    alpha,
    beta,
    k
):
    associations = []
    association_debug = []

    n_detections = len(detections_base)
    n_landmarks = len(landmarks)

    if n_detections == 0:
        return associations, [], association_debug

    if n_landmarks == 0:
        unassigned_detection_indices = list(range(n_detections))
        return associations, unassigned_detection_indices, association_debug

    cost_matrix, pair_data = build_association_cost_matrix(
        X,
        P,
        detections_base,
        landmarks,
        measurement_distribution_function,
        alpha,
        beta,
        k
    )

    gated_cost_matrix = build_gated_cost_matrix(
        cost_matrix,
        mahalanobis_gate
    )

    row_indices, column_indices = solve_global_assignment(
        gated_cost_matrix
    )

    associations, assigned_detection_indices, association_debug = select_gated_associations(
        row_indices,
        column_indices,
        cost_matrix,
        pair_data,
        landmarks,
        mahalanobis_gate
    )

    unassigned_detection_indices = compute_unassigned_detection_indices(
        n_detections,
        assigned_detection_indices
    )

    return associations, unassigned_detection_indices, association_debug


def get_landmark_position(X, landmark):
    idx = landmark["state_index"]

    return np.array([
        X[idx],
        X[idx + 1]
    ])


def find_nearest_landmark_euclidean(
    X,
    landmarks,
    landmark_position,
    max_distance
):
    best_landmark = None
    best_distance = None

    for landmark in landmarks:
        position = get_landmark_position(
            X,
            landmark
        )

        distance = float(
            np.linalg.norm(landmark_position - position)
        )

        if best_distance is None:
            best_distance = distance
            best_landmark = landmark
        elif distance < best_distance:
            best_distance = distance
            best_landmark = landmark

    if best_landmark is None:
        return None

    if best_distance > max_distance:
        return None

    return best_landmark


def build_measurement_debug_from_associations(
    X,
    P,
    detections_base,
    landmarks,
    associations,
    measurement_distribution_function,
    alpha,
    beta,
    k
):
    measurement_debug = []

    for association in associations:
        detection_index = association["detection_index"]
        landmark_index = association["landmark_list_index"]

        detection = detections_base[detection_index]
        landmark = landmarks[landmark_index]

        z_obs, R_measurement = get_detection_measurement(
            detection
        )

        distribution = measurement_distribution_function(
            X,
            P,
            R_measurement,
            landmark["state_index"],
            alpha,
            beta,
            k
        )

        if distribution is None:
            continue

        z_pred = distribution["z_pred"]
        S = distribution["S"]

        compatibility = compute_pair_compatibility(
            z_obs,
            z_pred,
            S
        )

        if compatibility is None:
            continue

        item = {
            "detection_index": detection_index,
            "landmark_id": landmark["id"],
            "state_index": landmark["state_index"],
            "z_obs": compatibility["z_obs"],
            "z_pred": compatibility["z_pred"],
            "innovation": compatibility["innovation"],
            "innovation_norm": compatibility["innovation_norm"],
            "mahalanobis_squared": compatibility["mahalanobis_squared"],
            "radius": detection["radius"],
            "geometric_rmse": detection["geometric_rmse"],
            "n_points": detection["n_points"]
        }

        measurement_debug.append(item)

    return measurement_debug