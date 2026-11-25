"""Calibração das rodas e escorregamento pela inclinação, sem ROS rodando."""

import math

import numpy as np

from agrobot_webots.slope_slip import slope_lateral_velocity
from agrobot_webots.wheel_calibration import (
    horizontal_velocity, roll_offset, roll_pitch_yaw, slip_gain)


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def test_roll_pitch_yaw_round_trip():
    for angles in ((0.02, -0.03, 1.0), (-0.05, 0.01, -2.5)):
        got = roll_pitch_yaw(quaternion(*angles))
        assert np.allclose(got, angles, atol=1e-9)


def test_horizontal_velocity_on_slope():
    # Subindo uma rampa de 5% para +x: a velocidade no frame do robô é toda
    # para a frente, a horizontal é cos(pitch) dela.
    pitch = -math.atan(0.05)
    forward, lateral = horizontal_velocity(
        quaternion(0.0, pitch, 0.0), (1.0, 0.0, 0.0))
    assert abs(forward - math.cos(pitch)) < 1e-9
    assert abs(lateral) < 1e-9
    # De través na encosta, escorregando para a direita (-y do robô) com o
    # rumo a 90 graus.
    forward, lateral = horizontal_velocity(
        quaternion(0.03, 0.0, math.pi / 2), (0.4, -0.02, 0.0))
    assert abs(forward - 0.4) < 1e-9
    assert abs(lateral + 0.02 * math.cos(0.03)) < 1e-9


def test_roll_offset_cancels_the_slope():
    rng = np.random.default_rng(0)
    slope = math.radians(1.5)
    offset = math.radians(-0.8)
    heading = np.array([0] * 300 + [2] * 200)
    roll = np.where(heading == 0, slope, -slope) + offset
    roll = roll + rng.normal(0, math.radians(0.1), len(roll))
    assert abs(roll_offset(roll, heading) - offset) < math.radians(0.05)
    # Só num sentido não dá para separar.
    assert roll_offset(roll[:300], heading[:300]) is None


def test_slip_gain_recovers_the_model():
    rng = np.random.default_rng(1)
    offset = math.radians(0.5)
    roll = rng.uniform(-0.04, 0.04, 1000) + offset
    speed = rng.uniform(0.2, 0.5, 1000)
    lateral = -0.6 * np.sin(roll - offset) * speed + rng.normal(0, 1e-3, 1000)
    gain, explained = slip_gain(lateral, roll, speed, offset)
    assert abs(gain + 0.6) < 0.02
    assert explained > 0.9


def test_slope_lateral_velocity():
    # Lado esquerdo mais alto (roll > 0), ganho negativo: escorrega para a
    # direita (vy < 0), na mesma proporção andando para trás.
    vy = slope_lateral_velocity(-0.6, math.radians(2.0), 0.0, 0.4)
    assert vy < 0.0
    assert abs(vy + 0.6 * math.sin(math.radians(2.0)) * 0.4) < 1e-12
    assert vy == slope_lateral_velocity(-0.6, math.radians(2.0), 0.0, -0.4)
    assert slope_lateral_velocity(-0.6, 0.01, 0.01, 0.4) == 0.0
