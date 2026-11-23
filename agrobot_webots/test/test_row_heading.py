"""Medida do rumo pelas fileiras (row_heading), sem ROS rodando."""

import math

import numpy as np

from agrobot_webots.row_heading import (
    heading_from_points, obstacle_points, row_yaw, wrap, yaw_from_rows)


def grove(robot_yaw, spacing=7.0, slope=0.05, seed=0):
    """
    Nuvem de um olival visto do robô: troncos em grade, chão inclinado.

    O robô fica no meio de uma entrelinha, com yaw robot_yaw em relação às
    fileiras (que vão ao longo de x no frame das fileiras). Devolve (N, 3)
    no base_link.
    """
    rng = np.random.default_rng(seed)
    points = []
    for i in range(-6, 7):
        for j in range(-4, 5):
            cx, cy = i * spacing + 1.3, j * spacing + spacing / 2
            for _ in range(40):
                a = rng.uniform(0, 2 * math.pi)
                z = rng.uniform(0.0, 2.5)
                points.append((cx + 0.2 * math.cos(a),
                               cy + 0.2 * math.sin(a), z))
    for _ in range(4000):
        x, y = rng.uniform(-40, 40, 2)
        points.append((x, y, 0.0))
    p = np.array(points)
    p[:, 2] += slope * p[:, 1] - 0.24 + rng.normal(0, 0.02, len(p))
    c, s = math.cos(-robot_yaw), math.sin(-robot_yaw)
    x = c * p[:, 0] - s * p[:, 1]
    y = s * p[:, 0] + c * p[:, 1]
    return np.column_stack([x, y, p[:, 2]])


def measure(robot_yaw, predicted, azimuth=0.0):
    obstacles = obstacle_points(grove(robot_yaw))
    angle_pred, k = row_yaw(predicted, azimuth)
    angle, quality, edge = heading_from_points(
        obstacles, angle_pred, math.radians(8))
    return yaw_from_rows(angle, azimuth, k), quality, edge


def test_ground_on_slope_is_not_an_obstacle():
    cloud = grove(0.0)
    obstacles = obstacle_points(cloud)
    # Só os troncos (40 pontos cada, a maioria acima de 0,3 m) sobram.
    assert len(obstacles) < 0.9 * 40 * 13 * 9
    assert len(obstacles) > 0.5 * 40 * 13 * 9


def test_measures_yaw_with_ekf_drift():
    for true_yaw, drift in ((0.0, 2.0), (math.radians(3), -4.0),
                            (math.pi, 5.0), (math.radians(-91), 3.0)):
        predicted = true_yaw + math.radians(drift)
        measured, quality, edge = measure(true_yaw, predicted)
        assert not edge
        assert quality > 1.2
        assert abs(math.degrees(wrap(measured - true_yaw))) < 0.3


def test_azimuth_offset():
    # Fileiras a 10 graus do odom: o yaw no odom é o relativo + 10.
    azimuth = math.radians(10)
    true_rel = math.radians(2)
    obstacles = obstacle_points(grove(true_rel))
    angle_pred, k = row_yaw(azimuth + true_rel + math.radians(2), azimuth)
    angle, _, _ = heading_from_points(obstacles, angle_pred, math.radians(8))
    measured = yaw_from_rows(angle, azimuth, k)
    assert abs(math.degrees(wrap(measured - azimuth - true_rel))) < 0.3


def test_no_rows_has_low_quality():
    rng = np.random.default_rng(1)
    cloud = np.column_stack([rng.uniform(-40, 40, (3000, 2)),
                             rng.uniform(0.5, 2.0, 3000)])
    _, quality, _ = heading_from_points(
        obstacle_points(cloud), 0.0, math.radians(8))
    assert quality < 1.2
