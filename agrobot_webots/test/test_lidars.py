"""Geometria dos LiDARs do driver e contas do teste da pessoa."""

import importlib.util
import math
import os

from agrobot_webots.agrobot_driver import (
    lidar_angles, lidar_directions, mask_self_hits, SAFETY_LIDAR_X,
    SELF_BOX_X, SELF_BOX_Y)
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_full_circle_rays_are_centered_in_their_slice():
    angles = lidar_angles(2 * math.pi, 360)
    step = math.radians(1.0)
    assert math.isclose(angles[0], math.pi - step / 2)
    assert math.isclose(angles[-1], -math.pi + step / 2)
    assert np.allclose(np.diff(angles), -step)


def test_partial_fov_keeps_the_edges():
    angles = lidar_angles(3.0, 300)
    assert math.isclose(angles[0], 1.5)
    assert math.isclose(angles[-1], -1.5)


def test_self_hits_become_nan_and_the_rest_stays():
    angles = np.array([0.0, math.pi, 2.0, math.pi])
    # Para a frente a 2 m (fora), para trás a 0,5 m (x = 0,15: dentro),
    # para trás e para a esquerda a 0,3 m (x = 0,53, y = 0,27: dentro), para
    # trás a 2 m (x = -1,35: fora).
    ranges = [2.0, 0.5, 0.3, 2.0]
    out = mask_self_hits(ranges, angles, SAFETY_LIDAR_X, SELF_BOX_X,
                         SELF_BOX_Y)
    assert out[0] == 2.0 and out[3] == 2.0
    assert np.isnan(out[1]) and np.isnan(out[2])


def test_inf_and_nan_pass_through_the_self_filter():
    angles = np.array([math.pi, 0.0])
    out = mask_self_hits([math.inf, math.nan], angles, SAFETY_LIDAR_X,
                         SELF_BOX_X, SELF_BOX_Y)
    assert math.isinf(out[0]) and np.isnan(out[1])


def test_lidar3d_directions_are_unit_and_ordered_top_to_bottom():
    directions = lidar_directions(2 * math.pi, 1800, math.radians(30), 16)
    assert directions.shape == (16 * 1800, 3)
    assert np.allclose(np.linalg.norm(directions, axis=1), 1.0, atol=1e-6)
    elevation = np.degrees(np.arcsin(directions[::1800, 2]))
    assert np.allclose(elevation, np.arange(15, -16, -2), atol=1e-4)


def test_person_clearance_to_the_robot_footprint():
    test = load('teste_pessoa', os.path.join(
        HERE, '..', 'scripts', 'teste_pessoa.py'))
    # Pessoa 1 m à frente do para-choque (x = 0,6): folga 1 - 0,25.
    assert math.isclose(test.clearance(0, 0, 0, 1.6, 0), 0.75)
    # Robô girado 90°: a frente aponta para +y.
    assert math.isclose(test.clearance(0, 0, math.pi / 2, 0, 1.6), 0.75,
                        abs_tol=1e-9)
    # Encostada na quina, por dentro do raio: contato.
    assert test.clearance(0, 0, 0, 0.7, 0.66) < 0


def test_person_walks_once_and_stops_at_the_end():
    person = load('pessoa_teste', os.path.join(
        HERE, '..', 'controllers', 'pessoa_teste', 'pessoa_teste.py'))
    points = [[0.0, -1.0], [0.0, 1.0]]
    ground = [0.1, 0.3]
    x, y, z, _ = person.position_on_path(points, ground, 1.0, once=True)
    assert (x, y) == (0.0, 0.0) and math.isclose(z, 0.2)
    x, y, _, _ = person.position_on_path(points, ground, 5.0, once=True)
    assert (x, y) == (0.0, 1.0)
    # Sem once, vai e volta: 3 m depois do início está a 1 m do fim, voltando.
    x, y, _, _ = person.position_on_path(points, ground, 3.0)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
