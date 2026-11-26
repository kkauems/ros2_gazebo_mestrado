"""Checagem dos waypoints pelas medidas do robô (waypoint_check), sem ROS."""

import math

from agrobot_webots import waypoint_check as wc

RES = 0.05
LENGTH, WIDTH, MARGIN = 1.2, 1.12, 0.05


def field_grid(rock=True):
    """
    Cabeceira leste da plantação, no frame da partida.

    Fileiras em y = -2,7, -0,9 e 0,9 até x = 10,5 (colmos a cada 0,3 m), a
    pedra grande em (11,8; -1,5) com raio 0,3 m e o resto livre, de x = 4 a
    14 e y = -4 a 2.
    """
    ox, oy, width, height = 4.0, -4.0, 200, 120
    data = [0] * (width * height)

    def mark(x, y):
        i = int((x - ox) / RES)
        j = int((y - oy) / RES)
        if 0 <= i < width and 0 <= j < height:
            data[j * width + i] = 100

    for row in (-2.7, -0.9, 0.9):
        x = 4.0
        while x <= 10.5 + 1e-6:
            mark(x, row)
            x += 0.3
    if rock:
        for k in range(-6, 7):
            for m in range(-6, 7):
                if math.hypot(k * RES, m * RES) <= 0.3:
                    mark(11.8 + k * RES, -1.5 + m * RES)
    return wc.Grid(data, width, height, RES, ox, oy)


FOOTPRINT = wc.Footprint(LENGTH, WIDTH, MARGIN)


def test_lane_center_is_free():
    grid = field_grid()
    status, _ = wc.check_waypoint(grid, FOOTPRINT, 6.0, -1.8, 0.0, 0.0)
    assert status == wc.OK


def test_point_on_a_row_is_blocked():
    grid = field_grid()
    status, _ = wc.check_waypoint(grid, FOOTPRINT, 6.0, -0.9, 0.0, 0.0)
    assert status == wc.BLOCKED
    found = wc.suggest(grid, FOOTPRINT, 6.0, -0.9, 0.0, 0.0)
    assert found is not None
    status, _ = wc.check_waypoint(grid, FOOTPRINT, *found, 0.0, 0.0)
    assert status == wc.OK


def test_old_east_headland_point_cannot_turn():
    """O ponto antigo (10,8; -1,8): cabe chegando, mas não gira para y = 0."""
    grid = field_grid()
    yaw_out = math.atan2(1.8, 0.5)
    status, angle = wc.check_waypoint(grid, FOOTPRINT, 10.8, -1.8, 0.0,
                                      yaw_out)
    assert status == wc.NO_TURN
    assert 0.0 < angle < yaw_out
    # Sem a pedra o mesmo giro passa.
    status, _ = wc.check_waypoint(field_grid(rock=False), FOOTPRINT, 10.8,
                                  -1.8, 0.0, yaw_out)
    assert status == wc.OK


def test_new_point_south_of_the_rock_is_valid():
    grid = field_grid()
    yaw_in = math.atan2(-2.75 + 1.8, 11.5 - 6.0)
    status, _ = wc.check_waypoint(grid, FOOTPRINT, 11.5, -2.75, yaw_in, 0.7)
    assert status == wc.OK


def test_outside_the_map_is_unknown():
    grid = field_grid()
    status, _ = wc.check_waypoint(grid, FOOTPRINT, 30.0, 0.0, 0.0, 0.0)
    assert status == wc.UNKNOWN


def test_turn_goes_the_short_way():
    angles = wc.turn_angles(math.radians(170), math.radians(-170))
    assert all(abs(math.cos(a) + 1.0) < 0.02 for a in angles)
    assert len(angles) == 5


def test_leg_uses_planned_arrival_heading():
    """Chegando ao ponto ao sul da pedra, o caminho vem em diagonal."""
    grid = field_grid()
    status, _, yaw_in = wc.check_leg(grid, FOOTPRINT, (6.0, -1.8), 11.6,
                                     -2.85, 0.7)
    assert status == wc.OK
    assert -1.4 < yaw_in < -0.2


def test_closed_pocket_is_unreachable():
    """Um ponto cercado por fileiras e pedra não tem caminho."""
    grid = field_grid()
    # Fecha a entrelinha y = -1,8 dos dois lados com colmos.
    for x in (4.2, 11.2):
        for k in range(-20, 21):
            i = int((x - grid.origin_x) / RES)
            j = int((-1.8 + k * RES - grid.origin_y) / RES)
            grid.data[j, i] = 100
    status, _, _ = wc.check_leg(grid, FOOTPRINT, (6.0, 0.0), 8.0, -1.8, 0.0)
    assert status == wc.UNREACHABLE



def lane_grid(specks, **kwargs):
    """
    Entrelinha y = 0 entre fileiras em y = ±0,9, como na grade do RTAB-Map.

    Cada planta ocupa 2 células (colmo e folha) a cada 0,3 m, com ±5 cm de
    sorteio, de x = 0 a 9; specks são células soltas na entrelinha.
    """
    ox, oy, width, height = -1.0, -2.0, 220, 80
    data = [0] * (width * height)

    def mark(i, j):
        if 0 <= i < width and 0 <= j < height:
            data[j * width + i] = 100

    jitter = [0, 1, -1, 1, 0, -1, -1, 1]
    for row in (-0.9, 0.9):
        j = int(round((row - oy) / RES))
        for k in range(31):
            i = int(round((k * 0.3 - ox) / RES)) + jitter[k % len(jitter)]
            mark(i, j)
            mark(i + 1, j)
    for x, y in specks:
        mark(int((x - ox) / RES), int((y - oy) / RES))
    return wc.Grid(data, width, height, RES, ox, oy, **kwargs)


def test_specks_in_the_lane_are_ignored():
    # Torrões e galhos na entrelinha viram células soltas na grade: sem
    # tirar os ciscos, o ponto no meio da entrelinha era recusado.
    specks = [(4.0, 0.3), (4.3, -0.3), (3.8, 0.0)]
    status, _ = wc.check_waypoint(lane_grid(specks), FOOTPRINT, 4.0, 0.0,
                                  0.0, 0.0)
    assert status == wc.BLOCKED
    raw = lane_grid(specks)
    clean = lane_grid(specks, speck_radius=0.3)
    for x, y in specks:
        i, j = clean.to_index(x, y)
        assert clean.data[j, i] == 0
    # Algumas pontas de planta (sorteio de 8 cm até a vizinha) também saem,
    # mas a fileira fica quase inteira.
    assert (clean.data >= 65).sum() > 0.8 * ((raw.data >= 65).sum() - 3)
    status, _ = wc.check_waypoint(clean, FOOTPRINT, 4.0, 0.0, 0.0, 0.0)
    assert status == wc.OK
    # As fileiras continuam: em cima delas o robô não cabe, e entre elas
    # ele não gira.
    status, _ = wc.check_waypoint(clean, FOOTPRINT, 4.0, 0.9, 0.0, 0.0)
    assert status == wc.BLOCKED
    status, _ = wc.check_waypoint(clean, FOOTPRINT, 4.0, 0.0, 0.0,
                                  math.pi / 2)
    assert status == wc.NO_TURN
