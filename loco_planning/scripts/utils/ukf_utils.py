#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

from utils.math_tools import wrapToPi


def symmetrize_covariance(P):
    return 0.5 * (P + P.T)


def safe_cholesky(P):
    P = symmetrize_covariance(P)

    n = P.shape[0]
    I = np.eye(n)

    jitter = 1.0e-12

    for i in range(8):
        try:
            return np.linalg.cholesky(P + jitter * I)
        except np.linalg.LinAlgError:
            jitter = jitter * 10.0

    return np.linalg.cholesky(P + jitter * I)


def wrap_angle_indices(vector, angle_indices):
    for idx in angle_indices:
        if idx is not None and 0 <= idx < vector.shape[0]:
            vector[idx] = wrapToPi(vector[idx])

    return vector


def compute_odometry_delta(odom_prev, odom_curr):
    x_odom_prev = odom_prev[0]
    y_odom_prev = odom_prev[1]
    theta_odom_prev = odom_prev[2]

    x_odom_curr = odom_curr[0]
    y_odom_curr = odom_curr[1]
    theta_odom_curr = odom_curr[2]

    dx_odom = x_odom_curr - x_odom_prev
    dy_odom = y_odom_curr - y_odom_prev
    dtheta_odom = wrapToPi(theta_odom_curr - theta_odom_prev)

    c_prev = math.cos(theta_odom_prev)
    s_prev = math.sin(theta_odom_prev)

    delta_forward = c_prev * dx_odom + s_prev * dy_odom
    delta_lateral = -s_prev * dx_odom + c_prev * dy_odom

    return np.array([delta_forward, delta_lateral, dtheta_odom])


def compute_odometry_delta_robot_pose(robot_state, odom_delta):
    x = robot_state[0]
    y = robot_state[1]
    theta = robot_state[2]

    delta_forward = odom_delta[0]
    delta_lateral = odom_delta[1]
    delta_theta = odom_delta[2]

    c = math.cos(theta)
    s = math.sin(theta)

    x_next = x + c * delta_forward - s * delta_lateral
    y_next = y + s * delta_forward + c * delta_lateral
    theta_next = wrapToPi(theta + delta_theta)

    return np.array([x_next, y_next, theta_next])


def odometry_motion_model(state, odom_delta):
    next_state = np.zeros(state.shape[0])

    robot_next = compute_odometry_delta_robot_pose(
        state[0:3],
        odom_delta
    )

    next_state[0] = robot_next[0]
    next_state[1] = robot_next[1]
    next_state[2] = robot_next[2]

    if state.shape[0] > 3:
        next_state[3:] = state[3:]

    return next_state


def compute_ukf_weights(n, alpha, beta, k):
    if alpha <= 0.0:
        raise ValueError("alpha must be > 0.")

    lambda_ = alpha * alpha * (n + k) - n
    tmp = n + lambda_

    if tmp <= 0.0:
        raise ValueError("n + lambda must be > 0.")

    Wm = np.zeros(2 * n + 1)
    Wc = np.zeros(2 * n + 1)

    Wm[0] = lambda_ / tmp
    Wc[0] = lambda_ / tmp + (1.0 - alpha * alpha + beta)

    for i in range(1, 2 * n + 1):
        Wm[i] = 1.0 / (2.0 * tmp)
        Wc[i] = 1.0 / (2.0 * tmp)

    return Wm, Wc, lambda_


def compute_sigma_points(x, P, lambda_, angle_indices):
    x = np.asarray(x)
    P = np.asarray(P)

    n = x.shape[0]
    gamma = math.sqrt(n + lambda_)

    S = safe_cholesky(P)

    sigma_points = np.zeros((n, 2 * n + 1))

    sigma_points[:, 0] = x.copy()
    wrap_angle_indices(sigma_points[:, 0], angle_indices)

    for i in range(n):
        sigma_points[:, i + 1] = x + gamma * S[:, i]
        sigma_points[:, n + i + 1] = x - gamma * S[:, i]

        wrap_angle_indices(sigma_points[:, i + 1], angle_indices)
        wrap_angle_indices(sigma_points[:, n + i + 1], angle_indices)

    return sigma_points


def compute_state_residual(state_i, state_mean):
    dx = state_i - state_mean

    if dx.shape[0] >= 3:
        dx[2] = wrapToPi(dx[2])

    return dx


def compute_predicted_mean(sigma_points_pred, Wm):
    n = sigma_points_pred.shape[0]
    num_sigma = sigma_points_pred.shape[1]

    x_pred = np.zeros(n)

    sin_sum = 0.0
    cos_sum = 0.0

    for i in range(num_sigma):
        for j in range(n):
            if j == 2:
                theta_i = sigma_points_pred[2, i]
                sin_sum = sin_sum + Wm[i] * math.sin(theta_i)
                cos_sum = cos_sum + Wm[i] * math.cos(theta_i)
            else:
                x_pred[j] = x_pred[j] + Wm[i] * sigma_points_pred[j, i]

    if n >= 3:
        x_pred[2] = math.atan2(sin_sum, cos_sum)

    return x_pred


def compute_predicted_covariance(sigma_points_pred, x_pred, Wc):
    n = x_pred.shape[0]
    num_sigma = sigma_points_pred.shape[1]

    P_pred = np.zeros((n, n))

    for i in range(num_sigma):
        dx = compute_state_residual(sigma_points_pred[:, i], x_pred)
        P_pred = P_pred + Wc[i] * np.outer(dx, dx)

    P_pred = symmetrize_covariance(P_pred)

    return P_pred


def augmented_prediction_state(x, P, odom_delta, P_odom_delta):
    n = x.shape[0]

    P_odom_delta = np.asarray(P_odom_delta)

    if P_odom_delta.shape == (3,):
        P_odom_delta = np.diag(P_odom_delta)
    elif P_odom_delta.shape != (3, 3):
        raise ValueError("P_odom_delta must be length 3 or 3x3.")

    P_odom_delta = symmetrize_covariance(P_odom_delta)

    x_aug = np.zeros(n + 3)
    x_aug[0:n] = x
    x_aug[n:n + 3] = odom_delta

    x_aug[2] = wrapToPi(x_aug[2])
    x_aug[n + 2] = wrapToPi(x_aug[n + 2])

    P_aug = np.zeros((n + 3, n + 3))
    P_aug[0:n, 0:n] = P
    P_aug[n:n + 3, n:n + 3] = P_odom_delta

    P_aug = symmetrize_covariance(P_aug)

    return x_aug, P_aug


def propagate_sigma_points(sigma_points_aug, n_state):
    num_sigma = sigma_points_aug.shape[1]

    sigma_points_pred = np.zeros((n_state, num_sigma))

    for i in range(num_sigma):
        x_sigma = sigma_points_aug[0:n_state, i]

        odom_delta_sigma = sigma_points_aug[n_state:n_state + 3, i].copy()
        odom_delta_sigma[2] = wrapToPi(odom_delta_sigma[2])

        sigma_points_pred[:, i] = odometry_motion_model(
            x_sigma,
            odom_delta_sigma
        )

    return sigma_points_pred


def ukf_prediction(x, P, P_odom_delta, alpha, beta, k, odom_prev, odom_curr):
    n_state = x.shape[0]

    odom_delta = compute_odometry_delta(odom_prev, odom_curr)

    x_aug, P_aug = augmented_prediction_state(
        x,
        P,
        odom_delta,
        P_odom_delta
    )

    n_aug = x_aug.shape[0]

    Wm_aug, Wc_aug, lambda_aug = compute_ukf_weights(
        n_aug,
        alpha,
        beta,
        k
    )

    sigma_points_aug = compute_sigma_points(
        x_aug,
        P_aug,
        lambda_aug,
        [2, n_state + 2]
    )

    sigma_points_pred = propagate_sigma_points(
        sigma_points_aug,
        n_state
    )

    x_pred = compute_predicted_mean(
        sigma_points_pred,
        Wm_aug
    )

    P_pred = compute_predicted_covariance(
        sigma_points_pred,
        x_pred,
        Wc_aug
    )

    return x_pred, P_pred


def landmark_measurement_model(state, landmark_state_index):
    x = state[0]
    y = state[1]
    theta = state[2]

    lx = state[landmark_state_index]
    ly = state[landmark_state_index + 1]

    dx = lx - x
    dy = ly - y

    c = math.cos(theta)
    s = math.sin(theta)

    z_pred = np.array([
        c * dx + s * dy,
        -s * dx + c * dy
    ])

    return z_pred


def measurement_residual(z_i, z_mean):
    return z_i - z_mean


def propagate_landmark_measurement_sigma_points(
    sigma_points,
    landmark_state_index
):
    num_sigma = sigma_points.shape[1]

    Z_sigma = np.zeros((2, num_sigma))

    for i in range(num_sigma):
        Z_sigma[:, i] = landmark_measurement_model(
            sigma_points[:, i],
            landmark_state_index
        )

    return Z_sigma


def compute_measurement_mean(Z_sigma, Wm):
    z_pred = np.zeros(Z_sigma.shape[0])

    for i in range(Z_sigma.shape[1]):
        z_pred = z_pred + Wm[i] * Z_sigma[:, i]

    return z_pred


def compute_update_covariances(
    sigma_points,
    Z_sigma,
    x_mean,
    z_mean,
    Wc
):
    n_state = x_mean.shape[0]
    num_sigma = Z_sigma.shape[1]

    S = np.zeros((2, 2))
    Pxz = np.zeros((n_state, 2))

    for i in range(num_sigma):
        dx = compute_state_residual(
            sigma_points[:, i],
            x_mean
        )

        dz = measurement_residual(
            Z_sigma[:, i],
            z_mean
        )

        S = S + Wc[i] * np.outer(dz, dz)
        Pxz = Pxz + Wc[i] * np.outer(dx, dz)

    S = symmetrize_covariance(S)

    return S, Pxz


def ukf_landmark_measurement_distribution(
    x,
    P,
    R_measurement,
    landmark_state_index,
    alpha,
    beta,
    k
):
    n_state = x.shape[0]

    R_measurement = np.asarray(R_measurement)

    if R_measurement.shape == (2,):
        R_measurement = np.diag(R_measurement)
    elif R_measurement.shape != (2, 2):
        raise ValueError("R_measurement must be length 2 or 2x2.")

    R_measurement = symmetrize_covariance(R_measurement)

    Wm, Wc, lambda_ = compute_ukf_weights(
        n_state,
        alpha,
        beta,
        k
    )

    sigma_points = compute_sigma_points(
        x,
        P,
        lambda_,
        [2]
    )

    Z_sigma = propagate_landmark_measurement_sigma_points(
        sigma_points,
        landmark_state_index
    )

    z_pred = compute_measurement_mean(
        Z_sigma,
        Wm
    )

    S, Pxz = compute_update_covariances(
        sigma_points,
        Z_sigma,
        x,
        z_pred,
        Wc
    )

    S = S + R_measurement
    S = symmetrize_covariance(S)

    return {
        "z_pred": z_pred,
        "S": S,
        "Pxz": Pxz
    }


def ukf_landmark_measurement_update(
    x,
    P,
    z_obs,
    R_measurement,
    landmark_state_index,
    alpha,
    beta,
    k
):
    distribution = ukf_landmark_measurement_distribution(
        x,
        P,
        R_measurement,
        landmark_state_index,
        alpha,
        beta,
        k
    )

    z_pred = distribution["z_pred"]
    S = distribution["S"]
    Pxz = distribution["Pxz"]

    innovation = measurement_residual(
        z_obs,
        z_pred
    )

    try:
        K = np.linalg.solve(S.T, Pxz.T).T
    except np.linalg.LinAlgError:
        return x, P, None

    x_upd = x + K @ innovation
    x_upd[2] = wrapToPi(x_upd[2])

    P_upd = P - K @ S @ K.T
    P_upd = symmetrize_covariance(P_upd)

    update_info = {
        "z_pred": z_pred,
        "innovation": innovation,
        "innovation_norm": np.linalg.norm(innovation),
        "S": S,
        "K": K
    }

    return x_upd, P_upd, update_info


def landmark_position_base2map(state, z_base):
    x = state[0]
    y = state[1]
    theta = state[2]

    c = math.cos(theta)
    s = math.sin(theta)

    lx = x + c * z_base[0] - s * z_base[1]
    ly = y + s * z_base[0] + c * z_base[1]

    return np.array([lx, ly])


def augmented_landmark_initialization_state(x, P, z_base, R_measurement):
    n = x.shape[0]

    R_measurement = np.asarray(R_measurement)

    if R_measurement.shape == (2,):
        R_measurement = np.diag(R_measurement)
    elif R_measurement.shape != (2, 2):
        raise ValueError("R_measurement must be length 2 or 2x2.")

    R_measurement = symmetrize_covariance(R_measurement)

    x_aug = np.zeros(n + 2)
    x_aug[0:n] = x
    x_aug[n:n + 2] = z_base

    x_aug[2] = wrapToPi(x_aug[2])

    P_aug = np.zeros((n + 2, n + 2))
    P_aug[0:n, 0:n] = P
    P_aug[n:n + 2, n:n + 2] = R_measurement

    P_aug = symmetrize_covariance(P_aug)

    return x_aug, P_aug


def propagate_landmark_initialization_sigma_points(
    sigma_points_aug,
    n_state
):
    num_sigma = sigma_points_aug.shape[1]

    sigma_points_new = np.zeros((n_state + 2, num_sigma))

    for i in range(num_sigma):
        x_sigma = sigma_points_aug[0:n_state, i]
        z_sigma = sigma_points_aug[n_state:n_state + 2, i]

        landmark_sigma = landmark_position_base2map(
            x_sigma,
            z_sigma
        )

        sigma_points_new[0:n_state, i] = x_sigma
        sigma_points_new[n_state:n_state + 2, i] = landmark_sigma

    return sigma_points_new


def augment_state_with_landmark(
    x,
    P,
    z_base,
    R_measurement,
    alpha,
    beta,
    k
):
    n_state = x.shape[0]

    x_aug, P_aug = augmented_landmark_initialization_state(
        x,
        P,
        z_base,
        R_measurement
    )

    n_aug = x_aug.shape[0]

    Wm_aug, Wc_aug, lambda_aug = compute_ukf_weights(
        n_aug,
        alpha,
        beta,
        k
    )

    sigma_points_aug = compute_sigma_points(
        x_aug,
        P_aug,
        lambda_aug,
        [2]
    )

    sigma_points_new = propagate_landmark_initialization_sigma_points(
        sigma_points_aug,
        n_state
    )

    x_new = np.zeros(n_state + 2)

    x_new[0:n_state] = x
    x_new[2] = wrapToPi(x_new[2])

    landmark_mean = np.zeros(2)

    for i in range(sigma_points_new.shape[1]):
        landmark_mean = landmark_mean + Wm_aug[i] * sigma_points_new[n_state:n_state + 2, i]

    x_new[n_state:n_state + 2] = landmark_mean

    P_new = compute_predicted_covariance(
        sigma_points_new,
        x_new,
        Wc_aug
    )

    landmark_state_index = n_state

    return x_new, P_new, landmark_state_index