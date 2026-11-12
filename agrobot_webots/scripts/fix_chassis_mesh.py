#!/usr/bin/env python3
"""
Gera agrobot_webots/meshes/chassis.stl (Webots) a partir do CAD original.

O chassi do CAD (agrobot_description/meshes/chassis.stl, o mesmo do Gazebo)
tem os motores e os eixos das rodas em x = +-0,2225 m e z = +0,0125 m (no
base_link), mas as rodas do robô (juntas do URDF e do agrobot.proto) ficam em
x = +-0,35 m e z = -0,02 m. Com isso as rodas apareciam soltas, 12,75 cm à
frente/atrás dos motores e 3,25 cm abaixo dos eixos.

Este script lê o CAD original (que não é alterado) e grava a cópia do Webots
com:
- os 4 motores (e os acoplamentos junto deles) movidos para o eixo das rodas:
  x = +-0,35 m e z = -0,02 m, presos por baixo das longarinas do chassi;
- os eixos do CAD removidos: os eixos que vão do motor até o cubo da roda
  ficam no agrobot.proto (Cylinder), com o mesmo comprimento nos quatro lados.

As unidades e a origem continuam as do CAD (mm), então o Transform do
agrobot.proto continua com scale 0.001. Rodar de novo dá o mesmo arquivo:

    python3 agrobot_webots/scripts/fix_chassis_mesh.py
"""

import os
import struct
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SOURCE = os.path.join(REPO, 'agrobot_description', 'meshes', 'chassis.stl')
TARGET = os.path.join(REPO, 'agrobot_webots', 'meshes', 'chassis.stl')

# Pose da malha no base_link (Transform do agrobot.proto, sem a correção de y,
# que é a mesma para a malha inteira): p_base = p_mm * 0,001 + OFFSET.
OFFSET = np.array([-0.5, -0.25, 0.05])

# Eixo dos motores no CAD e eixo das rodas no robô (base_link, m).
CAD_AXLE_X = 0.2225
CAD_AXLE_Z = 0.0125
WHEEL_X = 0.35
WHEEL_Z = -0.02

STL_DTYPE = np.dtype([('normal', '<3f4'), ('v', '<9f4'), ('attr', '<u2')])


def read_stl(path):
    with open(path, 'rb') as stl:
        data = stl.read()
    count = struct.unpack('<I', data[80:84])[0]
    return data[:80], np.frombuffer(data[84:84 + count * 50], dtype=STL_DTYPE).copy()


def write_stl(path, header, tris):
    with open(path, 'wb') as stl:
        stl.write(header)
        stl.write(struct.pack('<I', len(tris)))
        stl.write(tris.tobytes())


def components(tris):
    """Grupos de triângulos ligados por vértices em comum (peças do CAD)."""
    verts = np.round(tris['v'].reshape(-1, 3).astype(float), 3)
    _, index = np.unique(verts, axis=0, return_inverse=True)
    index = index.reshape(-1, 3)
    parent = list(range(index.max() + 1))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b, c in index:
        ra = find(a)
        for other in (b, c):
            ro = find(other)
            if ro != ra:
                parent[ro] = ra
    groups = defaultdict(list)
    for t, (a, _, _) in enumerate(index):
        groups[find(a)].append(t)
    return list(groups.values())


def main():
    header, tris = read_stl(SOURCE)
    pts = tris['v'].reshape(-1, 3, 3).astype(float) * 0.001 + OFFSET
    keep = np.ones(len(tris), dtype=bool)
    moved = removed = 0
    for group in components(tris):
        p = pts[group].reshape(-1, 3)
        lo, hi = p.min(0), p.max(0)
        center_x = (lo[0] + hi[0]) / 2
        near_axle = abs(abs(center_x) - CAD_AXLE_X) < 0.03 and hi[2] < 0.1
        if not near_axle:
            continue  # quadro e longarinas: ficam como estão
        if hi[0] - lo[0] < 0.02 and hi[1] - lo[1] > 0.15:
            keep[group] = False  # eixo fino e comprido do CAD
            removed += 1
            continue
        # motor ou acoplamento: vai para o eixo da roda
        shift_mm = np.array([np.sign(center_x) * (WHEEL_X - CAD_AXLE_X),
                             0.0, WHEEL_Z - CAD_AXLE_Z]) * 1000.0
        v = tris['v'][group].reshape(-1, 3, 3)
        tris['v'][group] = (v + shift_mm).reshape(-1, 9).astype('<f4')
        moved += 1
    out = tris[keep]
    write_stl(TARGET, header, out)
    print(f'{moved} peças movidas, {removed} eixos removidos, '
          f'{len(out)} triângulos -> {os.path.relpath(TARGET, REPO)}')


if __name__ == '__main__':
    main()
