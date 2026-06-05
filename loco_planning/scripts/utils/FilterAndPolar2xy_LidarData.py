#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np


def computeSigmaPhi(angle_increment):

    return abs(angle_increment) / math.sqrt(12.0)


def polarToCartesianCovariance(r, phi, sigma_r, sigma_phi):

    J = np.array([
        [np.cos(phi), -r * np.sin(phi)],
        [np.sin(phi),  r * np.cos(phi)]
    ])

    P_polar = np.array([
        [sigma_r * sigma_r, 0.0],
        [0.0, sigma_phi * sigma_phi]
    ])

    P_xy = J @ P_polar @ J.T
    P_xy = 0.5 * (P_xy + P_xy.T)

    return P_xy


def filterAndPolar2XYWithCovariance(msg, sigma_r, sigma_phi=None):

    if sigma_phi is None:
        Sigma_phi = computeSigmaPhi(msg.angle_increment)
    elif sigma_phi <= 0.0:
        Sigma_phi = computeSigmaPhi(msg.angle_increment)
    else:
        Sigma_phi = sigma_phi

    points = []
    covariances = []
    ranges = []
    angles = []

    angle = msg.angle_min

    for d in msg.ranges:
        if np.isfinite(d):
            if d >= msg.range_min and d <= msg.range_max:
                x = d * np.cos(angle)
                y = d * np.sin(angle)

                P_xy = polarToCartesianCovariance(
                    d,
                    angle,
                    sigma_r,
                    Sigma_phi
                )

                points.append([x, y])
                covariances.append(P_xy)
                ranges.append(d)
                angles.append(angle)

        angle = angle + msg.angle_increment

    return (
        np.array(points),
        np.array(covariances),
        np.array(ranges),
        np.array(angles)
    )