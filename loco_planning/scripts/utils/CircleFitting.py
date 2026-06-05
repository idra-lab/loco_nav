#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from scipy.optimize import least_squares


def fitCircleLeastSquares(points):
    points = np.asarray(points)

    if points.shape[0] < 3:
        return None

    x = points[:, 0]
    y = points[:, 1]

    A = np.column_stack((
        2.0 * x,
        2.0 * y,
        np.ones(points.shape[0])
    ))

    rhs = x * x + y * y

    try:
        result = np.linalg.lstsq(A, rhs, rcond=None)
        params = result[0]
    except np.linalg.LinAlgError:
        return None

    cx = params[0]
    cy = params[1]
    c = params[2]

    radius_sq = cx * cx + cy * cy + c

    if radius_sq <= 0.0:
        return None

    radius = math.sqrt(radius_sq)

    return np.array([cx, cy]), radius


def LSInitialGuess(points):

    result = fitCircleLeastSquares(points)

    if result is None:
        return None

    center, radius = result

    if radius <= 0.0:
        return None

    return center, radius


def buildNunezResidualAndJacobian(parameters, points, covariances):
    points = np.asarray(points)
    covariances = np.asarray(covariances)

    n_points = points.shape[0]

    cx = parameters[0]
    cy = parameters[1]
    radius = parameters[2]
    alphas = parameters[3:]

    residual = np.zeros(2 * n_points)
    jacobian = np.zeros((2 * n_points, 3 + n_points))

    i = 0
    while i < n_points:
        x_i = points[i, 0]
        y_i = points[i, 1]

        alpha_i = alphas[i]

        cos_a = math.cos(alpha_i)
        sin_a = math.sin(alpha_i)

        x_model = cx + radius * cos_a
        y_model = cy + radius * sin_a

        e_i = np.array([
            x_i - x_model,
            y_i - y_model
        ])

        J_e = np.zeros((2, 3 + n_points))

        J_e[0, 0] = -1.0
        J_e[1, 1] = -1.0

        J_e[0, 2] = -cos_a
        J_e[1, 2] = -sin_a

        J_e[0, 3 + i] = radius * sin_a
        J_e[1, 3 + i] = -radius * cos_a

        Sigma_i = covariances[i]
        Sigma_i = 0.5 * (Sigma_i + Sigma_i.T)

        try:
            L_i = np.linalg.cholesky(Sigma_i)
            f_i = np.linalg.solve(L_i, e_i)
            J_f = np.linalg.solve(L_i, J_e)
        except np.linalg.LinAlgError:
            return None, None

        residual[2 * i:2 * i + 2] = f_i
        jacobian[2 * i:2 * i + 2, :] = J_f

        i = i + 1

    return residual, jacobian


def fitCircleNunezFull(points, covariances, max_nfev=100):

    points = np.asarray(points)
    covariances = np.asarray(covariances)

    n_points = points.shape[0]

    if n_points < 4:

        return None

    if covariances.shape[0] != n_points:
        return None

    initial = LSInitialGuess(points)

    if initial is None:
        return None

    center0, radius0 = initial

    if radius0 <= 0.0:
        return None

    alphas0 = np.zeros(n_points)

    i = 0
    while i < n_points:
        dx = points[i, 0] - center0[0]
        dy = points[i, 1] - center0[1]
        alphas0[i] = math.atan2(dy, dx)
        i = i + 1

    parameters0 = np.zeros(3 + n_points)
    parameters0[0] = center0[0]
    parameters0[1] = center0[1]
    parameters0[2] = radius0
    parameters0[3:] = alphas0

    def residual_function(parameters):
        residual, jacobian = buildNunezResidualAndJacobian(
            parameters,
            points,
            covariances
        )

        if residual is None:
            return np.ones(2 * n_points) * 1.0e6

        return residual

    def jacobian_function(parameters):
        residual, jacobian = buildNunezResidualAndJacobian(
            parameters,
            points,
            covariances
        )

        if jacobian is None:
            return np.zeros((2 * n_points, 3 + n_points))

        return jacobian

    try:
        result = least_squares(
            residual_function,
            parameters0,
            jac=jacobian_function,
            method="lm",
            max_nfev=max_nfev
        )
    except Exception:
        return None

    if result.x[2] <= 0.0:
        return None

    parameters_hat = result.x

    residual_final, jacobian_final = buildNunezResidualAndJacobian(
        parameters_hat,
        points,
        covariances
    )

    if residual_final is None:
        return None

    if jacobian_final is None:
        return None

    residual_norm = float(residual_final.T @ residual_final)

    dof = n_points - 3

    if dof <= 0:
        return None

    s2 = residual_norm / float(dof)

    JTJ = jacobian_final.T @ jacobian_final

    try:
        P_all = s2 * np.linalg.inv(JTJ)
    except np.linalg.LinAlgError:
        return None

    P_all = 0.5 * (P_all + P_all.T)

    # cx, cy, r
    P_circle = P_all[0:3, 0:3]
    P_circle = 0.5 * (P_circle + P_circle.T)

    # cx, cy
    P_center = P_circle[0:2, 0:2]
    P_center = 0.5 * (P_center + P_center.T)

    center = np.array([
        parameters_hat[0],
        parameters_hat[1]
    ])

    radius = parameters_hat[2]

    geometric_rmse = computeGeometricCircleRmse(
        points,
        center,
        radius
    )

    output = {}
    output["center"] = center
    output["radius"] = radius
    output["P_center"] = P_center
    output["P_circle"] = P_circle
    output["P_all"] = P_all
    output["geometric_rmse"] = geometric_rmse
    output["s2"] = s2
    output["dof"] = dof
    output["residual_norm"] = residual_norm
    output["success"] = result.success
    output["message"] = result.message
    output["initialization"] = "algebraic_least_squares"

    return output


def fitCircleNunez(points, covariances, max_nfev=100):

    result = fitCircleNunezFull(
        points,
        covariances,
        max_nfev=max_nfev
    )

    if result is None:
        return None, None, None, None

    return (
        result["center"],
        result["radius"],
        result["geometric_rmse"],
        result["P_center"]
    )


def computeGeometricCircleRmse(points, center, radius):

    points = np.asarray(points)

    if points.shape[0] == 0:
        return 0.0

    total = 0.0

    i = 0

    while i < points.shape[0]:
        dx = points[i, 0] - center[0]
        dy = points[i, 1] - center[1]

        ri = math.sqrt(dx * dx + dy * dy)
        error = ri - radius

        total = total + error * error

        i = i + 1

    return math.sqrt(total / points.shape[0])


def computeCircleArcAngle(points, center):
    points = np.asarray(points)

    if points.shape[0] < 2:
        return 0.0

    angles = []

    i = 0

    while i < points.shape[0]:
        dx = points[i, 0] - center[0]
        dy = points[i, 1] - center[1]
        angles.append(math.atan2(dy, dx))
        i = i + 1

    angles = np.unwrap(np.array(angles))

    return math.degrees(np.max(angles) - np.min(angles))