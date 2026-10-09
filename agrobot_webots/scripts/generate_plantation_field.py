#!/usr/bin/env python3
"""
Gera o mundo Webots worlds/plantation_field.wbt (plantação, chão irregular).

O mundo é todo gerado por este script, com semente fixa: rodar de novo com os
mesmos argumentos dá o mesmo arquivo. Não edite o .wbt à mão; mude os
parâmetros abaixo (ou os argumentos de linha de comando) e gere de novo:

    python3 agrobot_webots/scripts/generate_plantation_field.py
    python3 agrobot_webots/scripts/generate_plantation_field.py --roughness 1.5

O que o mundo tem (eixo x ao longo das fileiras, z para cima):
- terreno ElevationGrid de 18 x 16 m com ondulação suave, rugosidade fina,
  camalhões sob as fileiras, trilhas de roda nas entrelinhas e buracos;
- 6 fileiras de plantas (CropPlant) de 9 m, 1,8 m entre fileiras, com
  falhas e plantas de alturas diferentes;
- torrões, pedras pequenas e galhos nas entrelinhas (passam por baixo do
  plano do LiDAR, o robô passa por cima), pedras grandes nas cabeceiras e
  nas margens;
- cerca de mourões em volta, atrito do solo menor que o padrão;
- o Agrobot na cabeceira oeste, de frente para a entrelinha central.
"""

import argparse
import math
import os
import random

# Terreno: grade de 18 x 16 m centrada na origem, pontos a cada 0,1 m.
GRID_X = 18.0
GRID_Y = 16.0
GRID_SPACING = 0.1

# Fileiras (ao longo de x, centradas na origem).
ROW_COUNT = 6
ROW_SPACING = 1.8       # entre fileiras (o robô tem 1,12 m de largura)
ROW_HALF_LENGTH = 4.5   # fileiras de x = -4,5 a 4,5 m
PLANT_SPACING = 0.3     # entre plantas na fileira
MISSING_PLANTS = 0.08   # fração de falhas na fileira

# Relevo (m).
# O LiDAR do Agrobot fica a ~8 cm do chão. Ondas de 5 a 12 m com 2 cm já
# fazem o feixe bater no chão à frente e o Nav2 marcar obstáculo na
# entrelinha (testado). Por isso a ondulação é longa (20 a 40 m): inclina o
# robô sem curvar o chão à frente dele.
UNDULATION = 0.05       # ondulação suave (morros largos)
UNDULATION_WAVELENGTH = (20.0, 40.0)
ROUGHNESS = 0.004       # rugosidade fina
ROUGHNESS_WAVELENGTH = (0.6, 2.0)
RIDGE_HEIGHT = 0.05     # camalhão sob a fileira
RIDGE_HALF_WIDTH = 0.3
RUT_DEPTH = 0.01        # trilhas de roda nas entrelinhas
RUT_HALF_WIDTH = 0.12
RUT_OFFSET = 0.48       # metade da bitola do Agrobot (0,96 m)
HOLE_DEPTH = (0.03, 0.05)  # buracos: um por entrelinha e um por cabeceira

# Obstáculos por entrelinha (todos abaixo do plano do LiDAR).
CLODS_PER_LANE = 5
STONES_PER_LANE = 2
BRANCHES_PER_LANE = 1

# Cerca em volta da lavoura: cabeceiras de 2,5 m depois das fileiras e
# margens de 1,5 m ao lado das fileiras de fora, mais 0,6 m até a cerca.
HEADLAND = 2.5
FENCE_X = ROW_HALF_LENGTH + HEADLAND + 0.6                      # 7,6 m
FENCE_Y = 0.5 * (ROW_COUNT - 1) * ROW_SPACING + 1.5 + 0.6      # 6,6 m
FENCE_POST_SPACING = 2.5

# Atrito roda-solo (o padrão do Webots é 1).
SOIL_FRICTION = 0.7

# Pose inicial do robô: cabeceira oeste, 1,5 m antes das fileiras, na
# entrelinha mais perto de y = 0, olhando para +x.
ROBOT_X = -(ROW_HALF_LENGTH + 1.5)
# base_link fica 0,24 m acima do chão (raio da roda 0,22 + eixo a -0,02).
ROBOT_HEIGHT = 0.26

EXTERNPROTO = 'https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects'


def smoothstep(edge0, edge1, value):
    """Rampa suave de 0 (em edge0) a 1 (em edge1)."""
    if edge0 == edge1:
        return 1.0 if value >= edge1 else 0.0
    t = min(max((value - edge0) / (edge1 - edge0), 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def bump(distance, half_width):
    """Perfil cos² com altura 1 no centro e 0 a partir de half_width."""
    if abs(distance) >= half_width:
        return 0.0
    return math.cos(0.5 * math.pi * distance / half_width) ** 2


def row_positions():
    """Coordenada y de cada fileira, centradas em y = 0."""
    first = -0.5 * (ROW_COUNT - 1) * ROW_SPACING
    return [first + i * ROW_SPACING for i in range(ROW_COUNT)]


def interrow_positions():
    """Coordenada y do centro de cada entrelinha."""
    rows = row_positions()
    return [0.5 * (a + b) for a, b in zip(rows, rows[1:])]


class Terrain:
    """Altura do chão h(x, y), com ondas de fase aleatória e semente fixa."""

    def __init__(self, rng, roughness_scale):
        """Sorteia as ondas, os buracos e calcula o deslocamento."""
        self.waves = []
        for _ in range(4):
            self.waves.append(self._wave(
                rng, *UNDULATION_WAVELENGTH, UNDULATION / 2.0))
        for _ in range(14):
            self.waves.append(self._wave(
                rng, *ROUGHNESS_WAVELENGTH,
                ROUGHNESS * roughness_scale / 2.0))
        # Buracos nas entrelinhas e nas cabeceiras: (x, y, profundidade, raio).
        self.holes = []
        for y in interrow_positions():
            x = rng.uniform(-ROW_HALF_LENGTH + 1.0, ROW_HALF_LENGTH - 1.0)
            self.holes.append((
                x, y + rng.uniform(-0.4, 0.4),
                rng.uniform(*HOLE_DEPTH) * roughness_scale,
                rng.uniform(0.3, 0.45)))
        for x in (-ROW_HALF_LENGTH - 1.3, ROW_HALF_LENGTH + 1.3):
            self.holes.append((
                x, rng.uniform(-3.0, 3.0),
                rng.uniform(*HOLE_DEPTH) * roughness_scale,
                rng.uniform(0.35, 0.5)))
        self.roughness_scale = roughness_scale
        self.rows = row_positions()
        self.interrows = interrow_positions()
        # Desloca tudo para cima: o terreno fica >= 0 e vai a 0 na borda,
        # onde encontra o chão plano de fora.
        self.offset = 0.0
        lowest = min(
            self.raw(-0.5 * GRID_X + i * GRID_SPACING,
                     -0.5 * GRID_Y + j * GRID_SPACING)
            for i in range(int(round(GRID_X / GRID_SPACING)) + 1)
            for j in range(int(round(GRID_Y / GRID_SPACING)) + 1))
        self.offset = -lowest + 0.005

    @staticmethod
    def _wave(rng, min_wavelength, max_wavelength, amplitude):
        wavelength = rng.uniform(min_wavelength, max_wavelength)
        direction = rng.uniform(0.0, math.pi)
        k = 2.0 * math.pi / wavelength
        return (k * math.cos(direction), k * math.sin(direction),
                rng.uniform(0.0, 2.0 * math.pi),
                amplitude * rng.uniform(0.6, 1.0))

    def raw(self, x, y):
        """Altura antes do deslocamento e da borda (pode ser negativa)."""
        height = 0.0
        for kx, ky, phase, amplitude in self.waves:
            height += amplitude * math.sin(kx * x + ky * y + phase)
        # Camalhões e trilhas só onde há fileira, com rampa nas pontas.
        along = smoothstep(ROW_HALF_LENGTH + 0.6, ROW_HALF_LENGTH, abs(x))
        if along > 0.0:
            ridge = max(bump(y - row, RIDGE_HALF_WIDTH) for row in self.rows)
            height += along * RIDGE_HEIGHT * ridge
            rut = 0.0
            for center in self.interrows:
                for side in (-RUT_OFFSET, RUT_OFFSET):
                    rut = max(rut, bump(y - center - side, RUT_HALF_WIDTH))
            height -= along * RUT_DEPTH * self.roughness_scale * rut
        for hx, hy, depth, radius in self.holes:
            distance = math.hypot(x - hx, y - hy)
            height -= depth * bump(distance, radius)
        return height

    def height(self, x, y):
        """Altura final do chão (m) no ponto (x, y) do mundo."""
        # Perto da borda da grade o relevo some (rampa de 1,2 m).
        edge = min(0.5 * GRID_X - abs(x), 0.5 * GRID_Y - abs(y))
        return smoothstep(0.0, 1.2, edge) * (self.raw(x, y) + self.offset)


def fmt(value):
    """Número curto para o .wbt."""
    text = f'{value:.4f}'.rstrip('0').rstrip('.')
    return '0' if text in ('', '-0') else text


def terrain_node(terrain):
    """Solid com o ElevationGrid do terreno."""
    nx = int(round(GRID_X / GRID_SPACING)) + 1
    ny = int(round(GRID_Y / GRID_SPACING)) + 1
    values = []
    # Ordem do ElevationGrid: x varia mais rápido, depois y.
    for j in range(ny):
        y = -0.5 * GRID_Y + j * GRID_SPACING
        for i in range(nx):
            x = -0.5 * GRID_X + i * GRID_SPACING
            values.append(fmt(terrain.height(x, y)))
    lines = []
    for start in range(0, len(values), 20):
        lines.append('          ' + ' '.join(values[start:start + 20]))
    heights = '\n'.join(lines)
    return f"""DEF FIELD_TERRAIN Solid {{
  translation {fmt(-0.5 * GRID_X)} {fmt(-0.5 * GRID_Y)} 0
  children [
    Shape {{
      appearance Soil {{
        textureTransform TextureTransform {{
          scale {fmt(GRID_X / 2.0)} {fmt(GRID_Y / 2.0)}
        }}
      }}
      geometry DEF FIELD_TERRAIN_GRID ElevationGrid {{
        xDimension {nx}
        xSpacing {fmt(GRID_SPACING)}
        yDimension {ny}
        ySpacing {fmt(GRID_SPACING)}
        height [
{heights}
        ]
      }}
    }}
  ]
  name "field_terrain"
  contactMaterial "soil"
  boundingObject USE FIELD_TERRAIN_GRID
  locked TRUE
}}
"""


def plant_nodes(terrain, rng):
    """Fileiras de CropPlant sobre os camalhões."""
    nodes = []
    count = int(round(2.0 * ROW_HALF_LENGTH / PLANT_SPACING)) + 1
    index = 0
    for row_index, row in enumerate(terrain.rows):
        for k in range(count):
            if rng.random() < MISSING_PLANTS:
                continue
            x = -ROW_HALF_LENGTH + k * PLANT_SPACING + rng.uniform(-0.05, 0.05)
            y = row + rng.uniform(-0.03, 0.03)
            z = terrain.height(x, y) - 0.01
            height = rng.uniform(0.9, 1.6)
            nodes.append(
                f'CropPlant {{\n'
                f'  translation {fmt(x)} {fmt(y)} {fmt(z)}\n'
                f'  rotation 0 0 1 {fmt(rng.uniform(-0.4, 0.4))}\n'
                f'  name "crop plant r{row_index} {k}"\n'
                f'  height {fmt(height)}\n'
                f'  stalkRadius {fmt(rng.uniform(0.016, 0.024))}\n'
                f'  leafLength {fmt(0.35 + 0.15 * height)}\n'
                f'  seed {index + 1}\n'
                f'}}\n')
            index += 1
    return nodes


def rock_node(name, x, y, z, rng, kind, scale, color):
    """Pedra (PROTO Rock do Webots), fixa, meio enterrada."""
    return (f'Rock {{\n'
            f'  translation {fmt(x)} {fmt(y)} {fmt(z)}\n'
            f'  rotation 0 0 1 {fmt(rng.uniform(0.0, 2.0 * math.pi))}\n'
            f'  name "{name}"\n'
            f'  type "{kind}"\n'
            f'  scale {fmt(scale)}\n'
            f'  color {color}\n'
            f'}}\n')


def branch_node(name, x, y, z, yaw, length, radius, first):
    """Galho caído na direção yaw: cilindro deitado, fixo."""
    appearance = ('DEF BRANCH_APPEARANCE PBRAppearance {\n'
                  '        baseColor 0.36 0.25 0.15\n'
                  '        roughness 1\n'
                  '        metalness 0\n'
                  '      }' if first else 'USE BRANCH_APPEARANCE')
    # O eixo do Cylinder é z; girar 90° em torno de (-sen, cos, 0) o deita
    # na direção (cos, sen, 0).
    return f"""Solid {{
  translation {fmt(x)} {fmt(y)} {fmt(z)}
  rotation {fmt(-math.sin(yaw))} {fmt(math.cos(yaw))} 0 1.5708
  children [
    Shape {{
      appearance {appearance}
      geometry Cylinder {{
        height {fmt(length)}
        radius {fmt(radius)}
        subdivision 8
      }}
    }}
  ]
  name "{name}"
  model "branch"
  boundingObject Cylinder {{
    height {fmt(length)}
    radius {fmt(radius)}
  }}
  locked TRUE
}}
"""


def obstacle_nodes(terrain, rng):
    """Torrões, pedras e galhos nas entrelinhas; pedras grandes fora delas."""
    nodes = []
    clod_color = '0.62 0.45 0.32'
    index = 0
    branches = 0
    for lane, center in enumerate(terrain.interrows):
        # Torrões: pedras chatas pequenas e marrons (3 a 5 cm de altura).
        for _ in range(CLODS_PER_LANE):
            x = rng.uniform(-ROW_HALF_LENGTH, ROW_HALF_LENGTH)
            y = center + rng.uniform(-0.6, 0.6)
            scale = rng.uniform(0.18, 0.28)
            nodes.append(rock_node(
                f'clod {index}', x, y, terrain.height(x, y) - 0.004, rng,
                'flat', scale, clod_color))
            index += 1
        # Pedras pequenas (topo 5 a 6,5 cm acima do chão).
        for _ in range(STONES_PER_LANE):
            x = rng.uniform(-ROW_HALF_LENGTH, ROW_HALF_LENGTH)
            y = center + rng.uniform(-0.6, 0.6)
            scale = rng.uniform(0.6, 0.8)
            nodes.append(rock_node(
                f'stone {index}', x, y,
                terrain.height(x, y) + 0.03 * scale, rng,
                'regular', scale, '1 1 1'))
            index += 1
        # Galhos atravessados na entrelinha.
        for _ in range(BRANCHES_PER_LANE):
            x = rng.uniform(-ROW_HALF_LENGTH + 0.5, ROW_HALF_LENGTH - 0.5)
            y = center + rng.uniform(-0.3, 0.3)
            radius = rng.uniform(0.02, 0.03)
            yaw = 0.5 * math.pi + rng.uniform(-0.6, 0.6)
            nodes.append(branch_node(
                f'branch {branches}', x, y,
                terrain.height(x, y) + radius - 0.006,
                yaw, rng.uniform(0.6, 1.0), radius, branches == 0))
            branches += 1
    # Pedras grandes (topo 15 a 23 cm acima do chão, o LiDAR vê): quatro nas
    # cabeceiras e duas nas margens, fora das fileiras e das entrelinhas.
    end = ROW_HALF_LENGTH
    side = terrain.rows[-1]
    spots = [(-(end + 1.1), -3.2), (-(end + 1.8), 2.4), (end + 1.3, -1.5),
             (end + 1.7, 3.8), (end / 3.0, -(side + 0.9)),
             (-end * 5.0 / 9.0, side + 1.0)]
    for k, (x, y) in enumerate(spots):
        nodes.append(rock_node(
            f'big rock {k}', x, y, terrain.height(x, y) - 0.02, rng,
            'flat', rng.uniform(1.0, 1.4), '0.9 0.88 0.85'))
    return nodes


def fence_nodes(terrain):
    """Mourões de madeira em volta da lavoura e dois fios de arame."""
    posts = []
    corners = [(-FENCE_X, -FENCE_Y), (FENCE_X, -FENCE_Y),
               (FENCE_X, FENCE_Y), (-FENCE_X, FENCE_Y)]
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        length = math.hypot(x1 - x0, y1 - y0)
        steps = int(math.ceil(length / FENCE_POST_SPACING))
        for k in range(steps):
            t = k / steps
            posts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    nodes = []
    for k, (x, y) in enumerate(posts):
        appearance = ('DEF POST_APPEARANCE PBRAppearance {\n'
                      '        baseColor 0.45 0.33 0.2\n'
                      '        roughness 1\n'
                      '        metalness 0\n'
                      '      }' if k == 0 else 'USE POST_APPEARANCE')
        nodes.append(f"""Solid {{
  translation {fmt(x)} {fmt(y)} {fmt(terrain.height(x, y) + 0.6)}
  children [
    Shape {{
      appearance {appearance}
      geometry Cylinder {{
        height 1.2
        radius 0.05
        subdivision 8
      }}
    }}
  ]
  name "fence post {k}"
  boundingObject Cylinder {{
    height 1.2
    radius 0.05
  }}
  locked TRUE
}}
""")
    # Fios: só visuais, bem acima do plano do LiDAR.
    wires = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        for z in (0.55, 1.05):
            wires.append(f"""    Pose {{
      translation {fmt(0.5 * (x0 + x1))} {fmt(0.5 * (y0 + y1))} {fmt(z)}
      rotation {fmt(-math.sin(yaw))} {fmt(math.cos(yaw))} 0 1.5708
      children [
        Shape {{
          appearance USE POST_APPEARANCE
          geometry Cylinder {{
            height {fmt(length)}
            radius 0.003
            subdivision 6
          }}
        }}
      ]
    }}""")
    nodes.append('Group {\n  children [\n' + '\n'.join(wires) + '\n  ]\n}\n')
    return nodes


def world_text(seed, roughness_scale):
    """Conteúdo completo do .wbt."""
    rng = random.Random(seed)
    terrain = Terrain(rng, roughness_scale)
    robot_y = min(terrain.interrows, key=abs)
    robot_z = terrain.height(ROBOT_X, robot_y) + ROBOT_HEIGHT
    header = f"""#VRML_SIM R2025a utf8
# Plantação com terreno não estruturado para o Agrobot.
# GERADO por agrobot_webots/scripts/generate_plantation_field.py
# (seed {seed}, roughness {fmt(roughness_scale)}). Não edite à mão: mude o script
# e gere de novo.

EXTERNPROTO "{EXTERNPROTO}/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "{EXTERNPROTO}/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "{EXTERNPROTO}/appearances/protos/Soil.proto"
EXTERNPROTO "{EXTERNPROTO}/appearances/protos/Grass.proto"
EXTERNPROTO "{EXTERNPROTO}/objects/rocks/protos/Rock.proto"
EXTERNPROTO "../protos/CropPlant.proto"
EXTERNPROTO "../protos/agrobot.proto"

WorldInfo {{
  info [
    "Plantacao com terreno nao estruturado (fileiras, camalhoes, torroes, pedras e galhos)"
  ]
  title "plantation_field"
  contactProperties [
    ContactProperties {{
      material1 "soil"
      coulombFriction [
        {fmt(SOIL_FRICTION)}
      ]
    }}
    # Roda sobre pedra gera mais de 10 pontos de contato (já vimos 32); com
    # o limite padrão (10) o Webots avisa no console a cada pedra.
    ContactProperties {{
      maxContactJoints 50
    }}
  ]
}}
Viewpoint {{
  orientation -0.33 0.18 0.93 2.2
  position 4.5 -14.0 11.0
}}
TexturedBackground {{
  texture "noon_cloudy_countryside"
}}
TexturedBackgroundLight {{
  texture "noon_cloudy_countryside"
}}
Solid {{
  translation 0 0 -0.002
  children [
    Shape {{
      appearance Grass {{
        type "maintained"
        textureTransform TextureTransform {{
          scale 30 30
        }}
      }}
      geometry Plane {{
        size 80 80
      }}
    }}
  ]
  name "outer_ground"
  boundingObject Plane {{
    size 80 80
  }}
  locked TRUE
}}
"""
    parts = [header, terrain_node(terrain)]
    parts.extend(plant_nodes(terrain, rng))
    parts.extend(obstacle_nodes(terrain, rng))
    parts.extend(fence_nodes(terrain))
    parts.append(f"""DEF AGROBOT agrobot {{
  translation {fmt(ROBOT_X)} {fmt(robot_y)} {fmt(robot_z)}
  name "agrobot"
  controller "<extern>"
}}
""")
    return ''.join(parts)


def main():
    """Lê os argumentos e escreve o .wbt."""
    here = os.path.dirname(os.path.abspath(__file__))
    default_output = os.path.join(
        here, '..', 'worlds', 'plantation_field.wbt')
    parser = argparse.ArgumentParser(
        description=__doc__.strip().splitlines()[0])
    parser.add_argument('--seed', type=int, default=7,
                        help='semente do sorteio (padrão 7)')
    parser.add_argument('--roughness', type=float, default=1.0,
                        help='escala da rugosidade fina, das trilhas de roda '
                             'e dos buracos (padrão 1.0; 0 deixa só a '
                             'ondulação e os camalhões)')
    parser.add_argument('--output', default=default_output,
                        help='arquivo .wbt de saída')
    args = parser.parse_args()
    if args.roughness < 0.0:
        parser.error('--roughness precisa ser >= 0')
    text = world_text(args.seed, args.roughness)
    with open(args.output, 'w', encoding='utf-8') as world:
        world.write(text)
    print(f'{os.path.normpath(args.output)}: {len(text) // 1024} KiB')


if __name__ == '__main__':
    main()
