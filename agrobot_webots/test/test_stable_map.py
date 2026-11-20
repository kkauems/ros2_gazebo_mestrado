"""Testes da grade conferida (stable_map.py), sem ROS."""

import math
import struct
import zlib

from agrobot_webots.stable_map import StableGrid, uncompress_cells
import numpy as np

# Duas fileiras de colmos a cada 0,3 m em y = ±0,9; o robô anda em y = 0.
POSTS = np.array([(x, y) for x in np.arange(0.0, 12.0, 0.3)
                  for y in (-0.9, 0.9)])


def local_grid(x, y, yaw, posts=POSTS, reach=8.0):
    """Grade local de um nó: colmos à frente e chão livre em volta."""
    c, s = math.cos(yaw), math.sin(yaw)
    rel = posts - (x, y)
    local = np.stack([c * rel[:, 0] + s * rel[:, 1],
                      -s * rel[:, 0] + c * rel[:, 1]], axis=-1)
    seen = (local[:, 0] > 0) & (np.hypot(*local.T) < reach)
    obstacles = local[seen]
    gx, gy = np.meshgrid(np.arange(0.025, reach, 0.05),
                         np.arange(-reach, reach, 0.05))
    free = np.stack([gx.ravel(), gy.ravel()], axis=-1)
    free = free[np.hypot(*free.T) < reach]
    if len(obstacles):
        d = np.min(np.hypot(free[:, None, 0] - obstacles[None, :, 0],
                            free[:, None, 1] - obstacles[None, :, 1]),
                   axis=1)
        free = free[d > 0.06]
    return obstacles.astype(np.float32), free.astype(np.float32)


def drive(grid, offset=(0.0, 0.0, 0.0), posts=POSTS, x_end=10.0):
    """Uma passada pela entrelinha, com a pose deslocada de offset."""
    accepted = []
    for x in np.arange(-1.0, x_end, 0.4):
        obstacles, free = local_grid(x, 0.0, 0.0, posts)
        _, ok = grid.add(x + offset[0], offset[1], offset[2],
                         obstacles, free)
        accepted.append(ok)
    return accepted


def occupied_near(grid, points, radius=0.1):
    occ = grid.occupancy() == 100
    i, j = np.nonzero(occ)
    cx = grid.origin[0] + (j + 0.5) * grid.res
    cy = grid.origin[1] + (i + 0.5) * grid.res
    return np.array([np.any(np.hypot(cx - px, cy - py) <= radius)
                     for px, py in points])


def ghosts(grid, posts=POSTS, radius=0.15):
    """Células ocupadas longe de qualquer colmo de verdade."""
    occ = grid.occupancy() == 100
    i, j = np.nonzero(occ)
    cx = grid.origin[0] + (j + 0.5) * grid.res
    cy = grid.origin[1] + (i + 0.5) * grid.res
    d = np.min(np.hypot(cx[:, None] - posts[None, :, 0],
                        cy[:, None] - posts[None, :, 1]), axis=1)
    return int(np.sum(d > radius))


def new_grid(**kw):
    grid = StableGrid(**kw)
    grid.reset(-10, -10, 20, 10)
    return grid


def test_uncompress_cells():
    points = np.array([[1.0, 2.0], [3.0, -4.0]], dtype=np.float32)
    cv_32fc2 = 5 + (2 - 1) * 8
    blob = zlib.compress(points.tobytes()) + struct.pack('<iii', 1, 2,
                                                         cv_32fc2)
    np.testing.assert_allclose(uncompress_cells(blob), points)
    assert uncompress_cells(b'').shape == (0, 2)


def test_misaligned_revisit_does_not_add_noise():
    # Segunda passada com 0,4 m de erro lateral: no RTAB-Map as fileiras
    # aparecem de novo ao lado; na grade conferida os nós são recusados.
    plain = new_grid(check=False)
    stable = new_grid()
    for grid in (plain, stable):
        drive(grid)
    first = ghosts(stable)
    plain_before = ghosts(plain)
    drive(plain, offset=(0.0, 0.4, 0.0))
    accepted = drive(stable, offset=(0.0, 0.4, 0.0))
    assert ghosts(plain) > plain_before + 20
    assert ghosts(stable) <= first + 2
    assert sum(accepted) < len(accepted) / 2
    assert stable.rejected > 0


def test_aligned_revisit_keeps_rows():
    stable = new_grid()
    drive(stable)
    accepted = drive(stable)
    assert all(accepted)
    assert occupied_near(stable, POSTS[POSTS[:, 0] < 9.0]).mean() > 0.95


def test_small_pose_error_does_not_duplicate_rows():
    # 0,15 m de erro: abaixo de min_shift, o nó entra, mas os pontos perto
    # das fileiras antigas são o mesmo obstáculo e não engordam a fileira.
    plain = new_grid(check=False)
    stable = new_grid()
    for grid in (plain, stable):
        drive(grid)
        drive(grid, offset=(0.0, 0.15, 0.0))
    assert ghosts(stable) < ghosts(plain)


def test_new_obstacle_enters_the_map():
    # Um monte de galhos no meio da entrelinha depois do mapeamento.
    stable = new_grid()
    drive(stable)
    pile = np.array([(6.0 + dx, dy) for dx in (0.0, 0.05, 0.1)
                     for dy in (-0.05, 0.0, 0.05)])
    drive(stable, posts=np.concatenate([POSTS, pile]), x_end=5.0)
    assert occupied_near(stable, pile).all()
