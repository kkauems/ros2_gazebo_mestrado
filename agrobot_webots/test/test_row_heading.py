"""Medida do rumo pelas fileiras (row_heading), sem ROS rodando."""

import math

import numpy as np

from agrobot_webots.row_heading import (
    heading_from_points, obstacle_points, row_offset, row_yaw, RowHeading,
    wrap, wrap_spacing, yaw_from_rows)


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


# --- Posição lateral pelas fileiras (lateral_correction:=rows) ---

def shifted_grove(x, y, yaw, seed=0):
    """
    Nuvem vista de um robô em (x, y, yaw) no frame das fileiras.

    Troncos da grade de 7 m em (7 i + 1,3; 7 j + 3,5): com o robô na
    origem, as fileiras ficam a 3,5 m de cada lado.
    """
    cloud = grove(0.0, seed=seed)
    p = cloud.copy()
    p[:, 0] -= x
    p[:, 1] -= y
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.column_stack([c * p[:, 0] - s * p[:, 1],
                            s * p[:, 0] + c * p[:, 1], p[:, 2]])


def test_row_offset_follows_lateral_position():
    for shift in (0.0, 0.4, -0.8, 1.7):
        obstacles = obstacle_points(shifted_grove(0.0, shift, 0.0))
        offset, concentration = row_offset(obstacles, 0.0, 7.0)
        assert abs(wrap_spacing(offset - (3.5 - shift), 7.0)) < 0.05
        assert concentration > 3.0


def test_row_offset_without_rows_is_flat():
    rng = np.random.default_rng(1)
    points = rng.uniform(-15, 15, (3000, 2))
    _, concentration = row_offset(points, 0.0, 7.0)
    assert concentration < 1.5


class FakeRowHeading(RowHeading):
    """Só a matemática da posição lateral, sem nó ROS."""

    def __init__(self):
        self.azimuth = 0.0
        self.spacing = (7.0, 7.0)
        self.lateral_range = 15.0
        self.min_concentration = 2.0
        self.max_offset = 1.0
        self.lateral_variance = 0.09
        self.gate = math.radians(8.0)
        self.bin_width = 0.2
        self.range_max = 40.0
        self.min_quality = 1.2
        self.calibration_samples = 1
        self.reference_distance = 0.0
        self.reference_start = None
        self.reference = [None, None]
        self.reference_samples = ([], [])
        self.lateral_sent = self.lateral_rejected = 0
        self.published = []

    def get_logger(self):
        class Quiet:
            def info(self, *args, **kwargs):
                pass
        return Quiet()

    def publish_position(self, x, y, nx, ny):
        self.published.append((x, y, nx, ny))


def test_lateral_error_is_measured_in_every_direction():
    node = FakeRowHeading()
    # Partida: robô na origem, EKF certo.
    node.calibrate_lateral(
        obstacle_points(shifted_grove(0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    assert abs(wrap_spacing(node.reference[0] - 3.5, 7.0)) < 0.05
    # Colunas: direção +y, normal (-1, 0): x = 1,3 vira -1,3.
    assert abs(wrap_spacing(node.reference[1] + 1.3, 7.0)) < 0.1
    # (x, y, yaw) verdadeiros e o erro do EKF (dx, dy).
    for true, error in (((20.0, 0.4, 0.0), (0.0, -0.4)),
                        ((30.0, -0.3, math.pi), (0.0, 0.3)),
                        ((40.0, 3.0, math.pi / 2), (0.5, 0.0)),
                        ((40.0, 10.0, -math.pi / 2), (-0.6, 0.0))):
        x, y, yaw = true
        ekf = (yaw, x + error[0], y + error[1])
        node.position = ekf[1:]
        node.published.clear()
        obstacles = obstacle_points(shifted_grove(x, y, yaw))
        predicted, k = row_yaw(yaw, node.azimuth)
        angle, _, _ = heading_from_points(obstacles, predicted, node.gate)
        node.measure_lateral(obstacles, angle, k, ekf)
        assert len(node.published) == 1, true
        px, py, nx, ny = node.published[0]
        # Corrige só na normal das linhas.
        assert abs((px - x) * nx + (py - y) * ny) < 0.05, true
        assert abs((px - ekf[1]) * ny - (py - ekf[2]) * nx) < 1e-9


def test_lateral_rejects_large_jumps():
    node = FakeRowHeading()
    node.calibrate_lateral(
        obstacle_points(shifted_grove(0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    node.position = (20.0, 0.0)
    obstacles = obstacle_points(shifted_grove(20.0, 2.0, 0.0))
    node.measure_lateral(obstacles, 0.0, 0, (0.0, 20.0, 0.0))
    assert node.published == []
    assert node.lateral_rejected == 1
