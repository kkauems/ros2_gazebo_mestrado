"""Partes sem ROS da interface web do operador."""

import math
import os

from agrobot_webots.operator_bridge import decimate, grid_to_gray, scan_points
from agrobot_webots.operator_web import inside, list_dir
from agrobot_webots.waypoint_mission import parse_points, safe_name
import numpy as np
import pytest


def test_grid_to_gray_flips_rows_and_colors_cells():
    # Linha 0 da grade é y mínimo: [livre, ocupado]; linha 1, desconhecido.
    gray = grid_to_gray([0, 100, -1, -1], width=2, height=2)
    assert gray.shape == (2, 2)
    assert gray.dtype == np.uint8
    assert gray[1, 0] > gray[0, 0] > gray[1, 1]   # livre > desconhecido > ocupado


def test_scan_points_in_global_frame():
    ranges = [1.0, float('inf'), 2.0]
    points = scan_points(ranges, -math.pi / 2, math.pi / 2, 8.0,
                         (1.0, 1.0, math.pi / 2), step=1)
    # Robô em (1, 1) olhando para +y: o raio da direita vai para +x e o da
    # esquerda para -x; o do meio (inf) fica de fora.
    assert points == pytest.approx([2.0, 1.0, -1.0, 1.0], abs=0.01)


def test_decimate_keeps_last_point():
    out = decimate(list(range(1000)), 10)
    assert len(out) == 10
    assert out[0] == 0 and out[-1] == 999


def test_parse_points_and_names():
    points = parse_points([{'x': 1, 'y': 2}, {'x': '3', 'y': 4, 'yaw': 0.5,
                                              'name': 'fim da fileira'}])
    assert points[0] == {'name': 'wp1', 'x': 1.0, 'y': 2.0, 'yaw_in': None}
    assert points[1]['name'] == 'fim-da-fileira'
    assert points[1]['yaw_in'] == 0.5
    with pytest.raises(ValueError):
        parse_points([{'x': float('nan'), 'y': 0}])
    assert safe_name('../x y', 'padrao') == 'x-y'
    assert safe_name('', 'padrao') == 'padrao'


def test_data_listing_stays_inside(tmp_path):
    (tmp_path / 'fotos' / 'missao1').mkdir(parents=True)
    (tmp_path / 'fotos' / 'missao1' / '000001.png').write_bytes(b'x' * 10)
    assert inside(str(tmp_path), '../etc/passwd') is None
    assert inside(str(tmp_path), 'fotos') == os.path.join(
        os.path.realpath(tmp_path), 'fotos')
    listing = list_dir(str(tmp_path), 'fotos/missao1')
    assert listing['entries'][0]['name'] == '000001.png'
    assert listing['entries'][0]['size'] == 10
    assert list_dir(str(tmp_path), '../..') is None
