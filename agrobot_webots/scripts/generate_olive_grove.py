#!/usr/bin/env python3
"""
Gera o mundo Webots worlds/olive_grove.wbt (olival tradicional do planalto).

Referência: olival em Vilarinho dos Galegos (Mogadouro), no planalto do Douro
Internacional: oliveiras em grade de ~7 m, em encosta suave, solo avermelhado
lavrado e pedregoso, vinha ao lado e estrada de terra.

O mundo é todo gerado por este script, com semente fixa: rodar de novo com os
mesmos argumentos dá o mesmo arquivo. Não edite o .wbt à mão; mude os
parâmetros abaixo (ou os argumentos de linha de comando) e gere de novo:

    python3 agrobot_webots/scripts/generate_olive_grove.py
    python3 agrobot_webots/scripts/generate_olive_grove.py --slope 0.03
    python3 agrobot_webots/scripts/generate_olive_grove.py --noise 0.5

O que o mundo tem (x para leste, ao longo das fileiras; y para norte, encosta
acima; z para cima):
- encosta que sobe para norte (4 a 5% no olival); por cima dela, um
  ElevationGrid de 100 x 84 m com chão irregular: ondulação larga, ruído em
  três escalas (12, 4 e 1,5 m) e montículos e covas espalhados;
- 70 lugares de oliveira (OliveTree) em grade de 7 x 7 m (10 por fileira,
  7 fileiras), com falhas e árvores novas;
- lascas de pedra e galhos caídos no chão (o robô passa por cima), pedras
  pequenas na linha das oliveiras;
- obstáculos que o LiDAR vê: montes de ramos de poda, montes de pedras
  (moroiços) e blocos, nas entrelinhas, nas cabeceiras e na linha das
  árvores, longe dos waypoints dos casos de estudo;
- vinha ao norte (4 fileiras com postes), estrada de terra ao sul e um
  tanque de rega com uma casinha a leste;
- o Agrobot na cabeceira oeste, de frente para a entrelinha y = -3,5.
"""

import argparse
import math
import os
import random

# Encosta: rampa suave (smoothstep) de sul para norte entre y = -HILL_HALF e
# y = HILL_HALF, com declive máximo SLOPE em y = 0; fora disso, plana. Ela
# vale para o terreno detalhado e para o chão de fora (grade grossa de
# OUTER_SIZE m), que fica OUTER_DROP abaixo para não aparecer por cima.
SLOPE = 0.05
HILL_HALF = 60.0
OUTER_SIZE = 400.0
OUTER_SPACING = 10.0
OUTER_DROP = 0.05

# Terreno detalhado: grade de 100 x 84 m centrada na origem. Na borda o
# relevo irregular vai a 0 (rampa de EDGE_FADE m) e sobra só a encosta.
# Células de 0,5 m bastam para o relevo abaixo (a menor escala é 1,5 m); com
# 0,25 m a simulação ficou ~25% mais lenta (câmeras renderizando mais chão).
GRID_X = 100.0
GRID_Y = 84.0
GRID_SPACING = 0.5
EDGE_FADE = 8.0

# Relevo irregular (m), somado à encosta. O LiDAR é um plano só, a ~13 cm do
# chão, e o Nav2 marca obstáculo até 5 m: quando o robô inclina num calombo,
# o plano pode bater no chão à frente. O que pesa é a curvatura: ondulação
# de poucos metros com mais de 2 a 3 cm já dava obstáculo falso a toda hora
# (testado com células de 6 m e 5 cm). Por isso o relevo forte é largo e o
# miúdo é raso. --noise escala o ruído e os montículos.
UNDULATION = 0.15
UNDULATION_WAVELENGTH = (50.0, 100.0)
NOISE_OCTAVES = ((12.0, 0.06), (4.0, 0.02), (1.5, 0.006))  # (célula, amplitude)
MOUNDS = 120            # montículos e covas
MOUND_HEIGHT = (0.02, 0.05)
MOUND_RADIUS = (1.2, 2.5)

# Olival: grade de 7 x 7 m, fileiras ao longo de x.
TREE_SPACING = 7.0
TREES_PER_ROW = 10      # x = -31,5 a 31,5
ROW_COUNT = 7           # y = -21 a 21
MISSING_TREES = 0.05
YOUNG_TREES = 0.12
FIELD_HALF_X = 38.0     # borda do olival (cabeceiras de ~6 m)
FIELD_HALF_Y = 25.0

# Pedras e galhos que o robô passa por cima.
CHIPS = 220             # lascas chatas (2 a 4 cm), em todo o olival
STONES = 40             # pedras pequenas (topo 5 a 6,5 cm), na linha das árvores
BRANCHES = 15           # galhos caídos (4 a 7 cm), nas entrelinhas

# Obstáculos que o LiDAR vê.
PRUNING_PILES = 10      # montes de ramos de poda, nas entrelinhas
STONE_HEAPS = 4         # moroiços: 2 nas entrelinhas e 2 nas cabeceiras
BIG_ROCKS = 12          # blocos (topo 15 a 23 cm): metade nas entrelinhas
# Os obstáculos ficam a pelo menos KEEP_CLEAR_RADIUS m dos waypoints dos
# casos de estudo (caso4_olival_ida_volta e caso5_olival_encosta, aqui no
# frame do mundo) e fora dos vãos por onde o caso5 atravessa as fileiras.
KEEP_CLEAR_POINTS = (
    (-36.0, -3.5), (-25.0, -3.5), (-14.0, -3.5), (-3.0, -3.5), (8.0, -3.5),
    (19.0, -3.5), (30.0, -3.5), (35.0, -3.5), (35.0, 3.5), (24.0, 3.5),
    (13.0, 3.5), (2.0, 3.5), (-9.0, 3.5), (-20.0, 3.5), (-31.0, 3.5),
    (-28.0, -3.5), (-28.0, 7.0), (-28.0, 17.5), (-17.5, 17.5), (-7.0, 17.5),
    (-7.0, 7.0), (-7.0, -3.5), (-7.0, -10.5), (-17.5, -10.5), (-28.0, -10.5))
KEEP_CLEAR_RADIUS = 2.5
CROSSING_GAPS = (-28.0, -7.0)   # x dos vãos que o caso5 usa

# Vinha ao norte do olival: fileiras ao longo de x.
VINE_ROWS = (28.5, 31.0, 33.5, 36.0)
VINE_HALF_LENGTH = 34.0
VINE_POST_SPACING = 5.0

# Estrada de terra ao sul.
ROAD_Y = (-30.5, -27.0)

# Tanque de rega e casinha, a leste do olival.
TANK = (45.0, -21.0, 3.0, 1.6)      # x, y, raio, altura
HUT = (45.0, -15.5, 3.0, 2.5, 2.6)  # x, y, largura, profundidade, altura

# Atrito roda-solo (o padrão do Webots é 1).
SOIL_FRICTION = 0.8

# Pose inicial do robô: cabeceira oeste, na entrelinha y = -3,5, olhando
# para +x.
ROBOT_X = -36.0
ROBOT_Y = -3.5
# base_link fica 0,24 m acima do chão (raio da roda 0,22 + eixo a -0,02).
ROBOT_HEIGHT = 0.26

EXTERNPROTO = 'https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects'


def smoothstep(edge0, edge1, value):
    """Rampa suave de 0 (em edge0) a 1 (em edge1)."""
    t = min(max((value - edge0) / (edge1 - edge0), 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def tree_rows():
    """Coordenada y de cada fileira de oliveiras."""
    first = -0.5 * (ROW_COUNT - 1) * TREE_SPACING
    return [first + j * TREE_SPACING for j in range(ROW_COUNT)]


def tree_columns():
    """Coordenada x de cada árvore na fileira."""
    first = -0.5 * (TREES_PER_ROW - 1) * TREE_SPACING
    return [first + k * TREE_SPACING for k in range(TREES_PER_ROW)]


class Terrain:
    """Altura do chão h(x, y): encosta mais relevo irregular."""

    def __init__(self, rng, slope=SLOPE, noise=1.0):
        """Sorteia as ondas, o ruído e os montículos."""
        # Altura total da encosta: smoothstep em 2 * HILL_HALF tem declive
        # máximo 1,5 * altura / (2 * HILL_HALF).
        self.rise = slope * 2.0 * HILL_HALF / 1.5
        self.waves = []
        for _ in range(3):
            self.waves.append(self._wave(
                rng, *UNDULATION_WAVELENGTH, UNDULATION / 2.0))
        # Ruído de valor: valores sorteados nos nós de grades de células
        # diferentes, interpolados com smoothstep.
        self.octaves = []
        for cell, amplitude in NOISE_OCTAVES:
            nx = int(GRID_X / cell) + 3
            ny = int(GRID_Y / cell) + 3
            values = [[rng.uniform(-1.0, 1.0) for _ in range(ny)]
                      for _ in range(nx)]
            self.octaves.append((cell, amplitude * noise, values))
        self.mounds = []
        for _ in range(MOUNDS):
            sign = 1.0 if rng.random() < 0.5 else -1.0
            self.mounds.append((
                rng.uniform(-0.5 * GRID_X, 0.5 * GRID_X),
                rng.uniform(-0.5 * GRID_Y, 0.5 * GRID_Y),
                rng.uniform(*MOUND_RADIUS),
                sign * rng.uniform(*MOUND_HEIGHT) * noise))
        # Desloca para cima, para o relevo irregular ficar >= 0.
        self.offset = (UNDULATION + sum(a for _, a, _ in self.octaves)
                       + MOUND_HEIGHT[1] * noise)

    @staticmethod
    def _wave(rng, min_wavelength, max_wavelength, amplitude):
        wavelength = rng.uniform(min_wavelength, max_wavelength)
        direction = rng.uniform(0.0, math.pi)
        k = 2.0 * math.pi / wavelength
        return (k * math.cos(direction), k * math.sin(direction),
                rng.uniform(0.0, 2.0 * math.pi),
                amplitude * rng.uniform(0.6, 1.0))

    def base(self, x, y):
        """Altura só da encosta (m), centrada em 0 no meio do olival."""
        return self.rise * (smoothstep(-HILL_HALF, HILL_HALF, y) - 0.5)

    def _noise(self, x, y):
        total = 0.0
        for cell, amplitude, values in self.octaves:
            u = (x + 0.5 * GRID_X) / cell + 1.0
            v = (y + 0.5 * GRID_Y) / cell + 1.0
            i, j = int(math.floor(u)), int(math.floor(v))
            fu, fv = smoothstep(0.0, 1.0, u - i), smoothstep(0.0, 1.0, v - j)
            i = min(max(i, 0), len(values) - 2)
            j = min(max(j, 0), len(values[0]) - 2)
            top = values[i][j] + fu * (values[i + 1][j] - values[i][j])
            bottom = (values[i][j + 1]
                      + fu * (values[i + 1][j + 1] - values[i][j + 1]))
            total += amplitude * (top + fv * (bottom - top))
        return total

    def relief(self, x, y):
        """Relevo irregular (m) por cima da encosta, já com a borda."""
        edge = min(0.5 * GRID_X - abs(x), 0.5 * GRID_Y - abs(y))
        fade = smoothstep(0.0, EDGE_FADE, edge)
        if fade == 0.0:
            return 0.0
        height = self.offset + self._noise(x, y)
        for kx, ky, phase, amplitude in self.waves:
            height += amplitude * math.sin(kx * x + ky * y + phase)
        for mx, my, radius, amplitude in self.mounds:
            d2 = (x - mx) ** 2 + (y - my) ** 2
            if d2 < 9.0 * radius * radius:
                height += amplitude * math.exp(-d2 / (radius * radius))
        return fade * height

    def height(self, x, y):
        """Altura do chão (m) no ponto (x, y) do mundo."""
        return self.base(x, y) + self.relief(x, y)


def fmt(value):
    """Número curto para o .wbt."""
    text = f'{value:.4f}'.rstrip('0').rstrip('.')
    return '0' if text in ('', '-0') else text


def grid_values(terrain, origin, size, spacing, lift=0.0):
    """Alturas de uma grade (x varia mais rápido), em linhas de 20."""
    values = []
    for j in range(size[1]):
        for i in range(size[0]):
            values.append(fmt(terrain.height(
                origin[0] + i * spacing[0], origin[1] + j * spacing[1]) + lift))
    lines = []
    for start in range(0, len(values), 20):
        lines.append('          ' + ' '.join(values[start:start + 20]))
    return '\n'.join(lines)


def terrain_node(terrain):
    """Solid com o ElevationGrid do terreno."""
    nx = int(round(GRID_X / GRID_SPACING)) + 1
    ny = int(round(GRID_Y / GRID_SPACING)) + 1
    heights = grid_values(terrain, (-0.5 * GRID_X, -0.5 * GRID_Y), (nx, ny),
                          (GRID_SPACING, GRID_SPACING))
    return f"""DEF GROVE_TERRAIN Solid {{
  translation {fmt(-0.5 * GRID_X)} {fmt(-0.5 * GRID_Y)} 0
  children [
    Shape {{
      appearance Soil {{
        color 1 0.8 0.68
        textureTransform TextureTransform {{
          scale {fmt(GRID_X / 2.5)} {fmt(GRID_Y / 2.5)}
        }}
      }}
      geometry DEF GROVE_TERRAIN_GRID ElevationGrid {{
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
  name "grove_terrain"
  contactMaterial "soil"
  boundingObject USE GROVE_TERRAIN_GRID
  locked TRUE
}}
"""


def outer_ground_node(terrain):
    """Chão de fora: grade grossa só com a encosta, um pouco mais baixa."""
    n = int(round(OUTER_SIZE / OUTER_SPACING)) + 1
    values = []
    for j in range(n):
        y = -0.5 * OUTER_SIZE + j * OUTER_SPACING
        for i in range(n):
            values.append(fmt(terrain.base(0.0, y) - OUTER_DROP))
    lines = ['          ' + ' '.join(values[k:k + 20])
             for k in range(0, len(values), 20)]
    heights = '\n'.join(lines)
    return f"""Solid {{
  translation {fmt(-0.5 * OUTER_SIZE)} {fmt(-0.5 * OUTER_SIZE)} 0
  children [
    Shape {{
      appearance Grass {{
        type "prickly"
        colorOverride 0.68 0.62 0.45
        textureTransform TextureTransform {{
          scale 100 100
        }}
      }}
      geometry DEF OUTER_GROUND_GRID ElevationGrid {{
        xDimension {n}
        xSpacing {fmt(OUTER_SPACING)}
        yDimension {n}
        ySpacing {fmt(OUTER_SPACING)}
        height [
{heights}
        ]
      }}
    }}
  ]
  name "outer_ground"
  boundingObject USE OUTER_GROUND_GRID
  locked TRUE
}}
"""


def road_node(terrain):
    """Estrada de terra: faixa só visual, 1,5 cm acima do chão."""
    x0 = -0.5 * GRID_X + EDGE_FADE
    length = GRID_X - 2.0 * EDGE_FADE
    width = ROAD_Y[1] - ROAD_Y[0]
    nx = int(round(length / 1.0)) + 1
    ny = int(round(width / 0.5)) + 1
    heights = grid_values(terrain, (x0, ROAD_Y[0]), (nx, ny), (1.0, 0.5),
                          0.015)
    return f"""Pose {{
  translation {fmt(x0)} {fmt(ROAD_Y[0])} 0
  children [
    Shape {{
      appearance SandyGround {{
        colorOverride 0.9 0.82 0.7
        textureTransform TextureTransform {{
          scale {fmt(length / 4.0)} 1
        }}
      }}
      geometry ElevationGrid {{
        xDimension {nx}
        xSpacing 1
        yDimension {ny}
        ySpacing 0.5
        height [
{heights}
        ]
      }}
    }}
  ]
}}
"""


def tree_nodes(terrain, rng):
    """Oliveiras (OliveTree) na grade, com falhas e árvores novas."""
    nodes = []
    positions = []
    seed = 0
    for j, row in enumerate(tree_rows()):
        for k, column in enumerate(tree_columns()):
            seed += 1
            if rng.random() < MISSING_TREES:
                continue
            x = column + rng.uniform(-0.4, 0.4)
            y = row + rng.uniform(-0.4, 0.4)
            positions.append((x, y))
            young = rng.random() < YOUNG_TREES
            scale = rng.uniform(0.55, 0.7) if young else rng.uniform(0.85, 1.15)
            trunk_height = rng.uniform(1.0, 1.25)
            nodes.append(
                f'OliveTree {{\n'
                f'  translation {fmt(x)} {fmt(y)} {fmt(terrain.height(x, y))}\n'
                f'  rotation 0 0 1 {fmt(rng.uniform(0.0, 2.0 * math.pi))}\n'
                f'  name "olive tree r{j} {k}"\n'
                f'  height {fmt(trunk_height + 2.3 * scale)}\n'
                f'  trunkHeight {fmt(trunk_height)}\n'
                f'  trunkRadius {fmt(0.14 * scale * rng.uniform(0.85, 1.2))}\n'
                f'  crownRadius {fmt(1.4 * scale)}\n'
                f'  seed {seed}\n'
                f'}}\n')
    return nodes, positions


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


def rock_nodes(terrain, rng, trees):
    """Lascas soltas e pedras pequenas (o robô passa por cima)."""
    nodes = []
    colors = ('0.78 0.75 0.7', '0.6 0.5 0.44', '0.7 0.62 0.55')
    rows = tree_rows()

    def clear_of_trees(x, y, radius):
        return all(math.hypot(x - tx, y - ty) >= radius for tx, ty in trees)

    # Lascas chatas (quartzo e xisto) em todo o olival: o robô passa por cima.
    # Com 3 a 5 cm, uma lasca sob a roda já inclinava o robô o bastante para o
    # plano do LiDAR tocar o chão alguns metros à frente.
    index = 0
    while index < CHIPS:
        x = rng.uniform(-FIELD_HALF_X + 2.0, FIELD_HALF_X - 2.0)
        y = rng.uniform(-FIELD_HALF_Y + 2.0, FIELD_HALF_Y - 2.0)
        if not clear_of_trees(x, y, 0.8):
            continue
        nodes.append(rock_node(
            f'chip {index}', x, y, terrain.height(x, y) - 0.004, rng, 'flat',
            rng.uniform(0.12, 0.22), rng.choice(colors)))
        index += 1
    # Pedras pequenas só perto da linha das árvores, fora das faixas por onde
    # o robô anda: uma pedra de 6 cm debaixo de uma roda inclina o robô e o
    # plano do LiDAR bate no chão à frente.
    count = 0
    while count < STONES:
        x = rng.uniform(-FIELD_HALF_X + 4.0, FIELD_HALF_X - 4.0)
        y = rng.choice(rows) + rng.uniform(-1.2, 1.2)
        if not clear_of_trees(x, y, 0.8):
            continue
        scale = rng.uniform(0.6, 0.8)
        nodes.append(rock_node(
            f'stone {count}', x, y, terrain.height(x, y) + 0.03 * scale, rng,
            'regular', scale, rng.choice(colors)))
        count += 1
    return nodes


def lane_rows():
    """Coordenada y do centro de cada entrelinha."""
    rows = tree_rows()
    return [0.5 * (a + b) for a, b in zip(rows, rows[1:])]


class Placer:
    """Sorteia lugares para obstáculos, longe das árvores e entre si."""

    def __init__(self, rng, trees):
        """Guarda o sorteio e as árvores."""
        self.rng = rng
        self.trees = trees
        self.taken = []

    def free(self, x, y, radius):
        """Diz se (x, y) está livre para um obstáculo de raio radius."""
        if any(math.hypot(x - tx, y - ty) < radius + 1.3
               for tx, ty in self.trees):
            return False
        if any(math.hypot(x - kx, y - ky) < radius + KEEP_CLEAR_RADIUS
               for kx, ky in KEEP_CLEAR_POINTS):
            return False
        if any(abs(x - gx) < radius + 1.5 for gx in CROSSING_GAPS) and any(
                abs(y - row) < 2.0 for row in tree_rows()):
            return False
        return all(math.hypot(x - ox, y - oy) >= radius + other + 2.0
                   for ox, oy, other in self.taken)

    def lane(self, radius, tries=500):
        """Lugar numa entrelinha (centro +-2 m)."""
        for _ in range(tries):
            x = self.rng.uniform(-FIELD_HALF_X + 6.0, FIELD_HALF_X - 6.0)
            y = self.rng.choice(lane_rows()) + self.rng.uniform(-2.0, 2.0)
            if self.free(x, y, radius):
                self.taken.append((x, y, radius))
                return x, y
        raise RuntimeError('sem lugar livre para um obstáculo')

    def tree_line(self, radius, tries=500):
        """Lugar na linha das árvores, no vão entre duas oliveiras."""
        for _ in range(tries):
            x = (self.rng.choice(tree_columns()[:-1]) + 0.5 * TREE_SPACING
                 + self.rng.uniform(-0.5, 0.5))
            y = self.rng.choice(tree_rows()) + self.rng.uniform(-0.4, 0.4)
            if self.free(x, y, radius):
                self.taken.append((x, y, radius))
                return x, y
        raise RuntimeError('sem lugar livre para um obstáculo')

    def headland(self, radius, tries=500):
        """Lugar nas cabeceiras (entre a última árvore e a borda)."""
        for _ in range(tries):
            side = self.rng.choice((-1.0, 1.0))
            x = side * self.rng.uniform(FIELD_HALF_X - 5.0, FIELD_HALF_X - 1.0)
            y = self.rng.uniform(-FIELD_HALF_Y + 3.0, FIELD_HALF_Y - 3.0)
            if self.free(x, y, radius):
                self.taken.append((x, y, radius))
                return x, y
        raise RuntimeError('sem lugar livre para um obstáculo')


def branch_node(name, x, y, z, yaw, length, radius, appearance):
    """Galho deitado na direção yaw: cilindro com colisão, fixo."""
    # O eixo do Cylinder é z; girar 90 graus em torno de (-sen, cos, 0) o
    # deita na direção (cos, sen, 0).
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


def pruning_pile_node(name, x, y, terrain, rng):
    """Monte de ramos de poda: galhos cruzados e tufos de folhas secas."""
    length = rng.uniform(1.6, 2.6)
    width = rng.uniform(0.9, 1.4)
    height = rng.uniform(0.45, 0.75)
    yaw = rng.uniform(0.0, math.pi)
    ground = min(terrain.height(x + dx, y + dy)
                 for dx in (-0.8, 0.0, 0.8) for dy in (-0.8, 0.0, 0.8))
    children = []
    for k in range(18):
        # Galho no referencial do monte: centro, direção e inclinação.
        bx = rng.uniform(-0.35, 0.35) * length
        by = rng.uniform(-0.35, 0.35) * width
        bz = rng.uniform(0.08, 0.85) * height
        angle = rng.uniform(-0.5, 0.5) + (0.0 if k % 3 else 1.2)
        tilt = rng.uniform(-0.35, 0.35)
        size = rng.uniform(0.5, 0.95) * length
        # Eixo do galho: (cos a cos t, sen a cos t, sen t); o Cylinder (eixo
        # z) é girado para ele em torno de z x eixo.
        ax = math.cos(angle) * math.cos(tilt)
        ay = math.sin(angle) * math.cos(tilt)
        az = math.sin(tilt)
        norm = math.hypot(ax, ay) or 1.0
        rotation = (f'{fmt(-ay / norm)} {fmt(ax / norm)} 0 '
                    f'{fmt(math.acos(max(-1.0, min(1.0, az))))}')
        appearance = ('DEF PRUNING_WOOD PBRAppearance {\n'
                      '            baseColor 0.4 0.33 0.25\n'
                      '            roughness 1\n'
                      '            metalness 0\n'
                      '          }' if k == 0 else 'USE PRUNING_WOOD')
        children.append(f"""    Pose {{
      translation {fmt(bx)} {fmt(by)} {fmt(bz)}
      rotation {rotation}
      children [
        Shape {{
          appearance {appearance}
          geometry Cylinder {{
            height {fmt(size)}
            radius {fmt(rng.uniform(0.012, 0.035))}
            subdivision 6
          }}
        }}
      ]
    }}""")
    for k in range(5):
        tone = rng.uniform(0.85, 1.1)
        color = ' '.join(fmt(c * tone) for c in (0.45, 0.47, 0.34))
        radius = rng.uniform(0.18, 0.32)
        px = rng.uniform(-0.3, 0.3) * length
        py = rng.uniform(-0.3, 0.3) * width
        pz = rng.uniform(0.35, 0.8) * height
        children.append(f"""    Pose {{
      translation {fmt(px)} {fmt(py)} {fmt(pz)}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor {color}
            roughness 1
            metalness 0
          }}
          geometry Sphere {{
            radius {fmt(radius)}
            subdivision 1
          }}
        }}
      ]
    }}""")
    return (f'Solid {{\n'
            f'  translation {fmt(x)} {fmt(y)} {fmt(ground)}\n'
            f'  rotation 0 0 1 {fmt(yaw)}\n'
            f'  children [\n' + '\n'.join(children) + '\n  ]\n'
            f'  name "{name}"\n'
            f'  model "pruning pile"\n'
            f'  boundingObject Pose {{\n'
            f'    translation 0 0 {fmt(0.5 * height)}\n'
            f'    children [\n'
            f'      Box {{\n'
            f'        size {fmt(0.8 * length)} {fmt(0.8 * width)} {fmt(height)}\n'
            f'      }}\n'
            f'    ]\n'
            f'  }}\n'
            f'  locked TRUE\n'
            f'}}\n')


def obstacle_nodes(terrain, rng, trees):
    """Galhos caídos, montes de poda, moroiços e blocos."""
    nodes = []
    placer = Placer(rng, trees)
    colors = ('0.78 0.75 0.7', '0.6 0.5 0.44', '0.7 0.62 0.55')
    # Galhos caídos nas entrelinhas (4 a 7 cm de altura): o robô passa por
    # cima. Não entram no Placer (não precisam de folga).
    lanes = lane_rows()
    for k in range(BRANCHES):
        x = rng.uniform(-FIELD_HALF_X + 4.0, FIELD_HALF_X - 4.0)
        y = rng.choice(lanes) + rng.uniform(-2.5, 2.5)
        radius = rng.uniform(0.02, 0.035)
        appearance = ('DEF BRANCH_WOOD PBRAppearance {\n'
                      '        baseColor 0.36 0.28 0.2\n'
                      '        roughness 1\n'
                      '        metalness 0\n'
                      '      }' if k == 0 else 'USE BRANCH_WOOD')
        nodes.append(branch_node(
            f'branch {k}', x, y, terrain.height(x, y) + radius - 0.006,
            rng.uniform(0.0, math.pi), rng.uniform(0.8, 1.6), radius,
            appearance))
    # Montes de ramos de poda nas entrelinhas.
    for k in range(PRUNING_PILES):
        x, y = placer.lane(1.3)
        nodes.append(pruning_pile_node(f'pruning pile {k}', x, y, terrain,
                                       rng))
    # Moroiços (montes de pedras soltas): 7 a 10 pedras empilhadas.
    for k in range(STONE_HEAPS):
        x, y = placer.lane(1.0) if k < 2 else placer.headland(1.0)
        count = rng.randint(7, 10)
        for n in range(count):
            angle = rng.uniform(0.0, 2.0 * math.pi)
            distance = rng.uniform(0.0, 0.6) if n < count - 2 else 0.0
            px = x + distance * math.cos(angle)
            py = y + distance * math.sin(angle)
            lift = 0.0 if n < count - 2 else 0.15 + 0.1 * (n - count + 2)
            nodes.append(rock_node(
                f'stone heap {k} {n}', px, py,
                terrain.height(px, py) + lift, rng, 'regular',
                rng.uniform(2.2, 3.2), rng.choice(colors)))
    # Blocos (topo 15 a 23 cm): metade nas entrelinhas, metade na linha das
    # árvores, entre duas oliveiras.
    for k in range(BIG_ROCKS):
        x, y = placer.lane(0.6) if k % 2 == 0 else placer.tree_line(0.6)
        nodes.append(rock_node(
            f'big rock {k}', x, y, terrain.height(x, y) - 0.02, rng, 'flat',
            rng.uniform(1.0, 1.4), colors[0]))
    return nodes


def vineyard_nodes(terrain, rng):
    """Vinha: em cada fileira, postes com colisão e a sebe das videiras."""
    nodes = []
    posts = int(round(2.0 * VINE_HALF_LENGTH / VINE_POST_SPACING)) + 1
    for r, y in enumerate(VINE_ROWS):
        children = []
        bounds = []
        for p in range(posts):
            x = -VINE_HALF_LENGTH + p * VINE_POST_SPACING
            z = terrain.height(x, y) + 0.55
            appearance = 'USE VINE_POST'
            if r == 0 and p == 0:
                appearance = ('DEF VINE_POST PBRAppearance {\n'
                              '            baseColor 0.42 0.32 0.22\n'
                              '            roughness 1\n'
                              '            metalness 0\n'
                              '          }')
            children.append(f"""    Pose {{
      translation {fmt(x)} {fmt(y)} {fmt(z)}
      children [
        Shape {{
          appearance {appearance}
          geometry Cylinder {{
            height 1.3
            radius 0.04
            subdivision 6
          }}
        }}
      ]
    }}""")
            bounds.append(f"""      Pose {{
        translation {fmt(x)} {fmt(y)} {fmt(z)}
        children [
          Cylinder {{
            height 1.3
            radius 0.04
          }}
        ]
      }}""")
            if p == posts - 1:
                continue
            # Sebe entre dois postes (videiras podadas), inclinada como o
            # chão; altura, largura e tom variam (só visual).
            xm = x + 0.5 * VINE_POST_SPACING
            height = rng.uniform(0.55, 0.8)
            zm = terrain.height(xm, y) + 0.3 + 0.5 * height
            pitch = -math.atan2(terrain.height(x + VINE_POST_SPACING, y)
                                - terrain.height(x, y), VINE_POST_SPACING)
            tone = rng.uniform(0.85, 1.15)
            color = ' '.join(fmt(min(c * tone, 1.0))
                             for c in (0.45, 0.33, 0.22))
            size = (f'{fmt(VINE_POST_SPACING - 0.2)} '
                    f'{fmt(rng.uniform(0.3, 0.45))} {fmt(height)}')
            children.append(f"""    Pose {{
      translation {fmt(xm)} {fmt(y)} {fmt(zm)}
      rotation 0 1 0 {fmt(pitch)}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor {color}
            roughness 1
            metalness 0
          }}
          geometry Box {{
            size {size}
          }}
        }}
      ]
    }}""")
        nodes.append(
            'Solid {\n  children [\n' + '\n'.join(children) + '\n  ]\n'
            f'  name "vine row {r}"\n'
            '  model "vine row"\n'
            '  boundingObject Group {\n    children [\n'
            + '\n'.join(bounds) + '\n    ]\n  }\n  locked TRUE\n}\n')
    return nodes


def tank_nodes(terrain):
    """Tanque de rega circular e uma casinha de bombas."""
    x, y, radius, height = TANK
    ground = min(terrain.height(x + dx, y + dy)
                 for dx in (-radius, 0, radius) for dy in (-radius, 0, radius))
    tank = f"""Solid {{
  translation {fmt(x)} {fmt(y)} {fmt(ground + 0.5 * height - 0.2)}
  children [
    Shape {{
      appearance DEF CONCRETE Roughcast {{
        colorOverride 0.72 0.7 0.66
        textureTransform TextureTransform {{
          scale 4 1
        }}
      }}
      geometry Cylinder {{
        height {fmt(height + 0.4)}
        radius {fmt(radius)}
        subdivision 32
      }}
    }}
    Pose {{
      translation 0 0 {fmt(0.5 * height + 0.19)}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.2 0.25 0.22
            roughness 0.2
            metalness 0
          }}
          geometry Cylinder {{
            height 0.02
            radius {fmt(radius - 0.15)}
            subdivision 32
          }}
        }}
      ]
    }}
  ]
  name "irrigation tank"
  boundingObject Cylinder {{
    height {fmt(height + 0.4)}
    radius {fmt(radius)}
  }}
  locked TRUE
}}
"""
    x, y, width, depth, height = HUT
    ground = min(terrain.height(x + dx, y + dy)
                 for dx in (-0.5 * width, 0.5 * width)
                 for dy in (-0.5 * depth, 0.5 * depth))
    hut = f"""Solid {{
  translation {fmt(x)} {fmt(y)} {fmt(ground + 0.5 * height - 0.2)}
  children [
    Shape {{
      appearance USE CONCRETE
      geometry Box {{
        size {fmt(width)} {fmt(depth)} {fmt(height + 0.4)}
      }}
    }}
  ]
  name "pump house"
  boundingObject Box {{
    size {fmt(width)} {fmt(depth)} {fmt(height + 0.4)}
  }}
  locked TRUE
}}
"""
    return [tank, hut]


def world_text(seed, slope=SLOPE, noise=1.0):
    """Conteúdo completo do .wbt."""
    # Sorteios separados: mudar o relevo não muda onde ficam as árvores e
    # os obstáculos.
    terrain = Terrain(random.Random(seed), slope, noise)
    rng = random.Random(seed + 1000)
    robot_z = terrain.height(ROBOT_X, ROBOT_Y) + ROBOT_HEIGHT
    header = f"""#VRML_SIM R2025a utf8
# Olival tradicional do planalto (Douro Internacional) para o Agrobot.
# GERADO por agrobot_webots/scripts/generate_olive_grove.py
# (seed {seed}, slope {fmt(slope)}, noise {fmt(noise)}). Não edite à mão:
# mude o script e gere de novo.

EXTERNPROTO "{EXTERNPROTO}/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "{EXTERNPROTO}/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "{EXTERNPROTO}/appearances/protos/Soil.proto"
EXTERNPROTO "{EXTERNPROTO}/appearances/protos/Grass.proto"
EXTERNPROTO "{EXTERNPROTO}/appearances/protos/SandyGround.proto"
EXTERNPROTO "{EXTERNPROTO}/appearances/protos/Roughcast.proto"
EXTERNPROTO "{EXTERNPROTO}/objects/rocks/protos/Rock.proto"
EXTERNPROTO "../protos/OliveTree.proto"
EXTERNPROTO "../protos/agrobot.proto"

WorldInfo {{
  info [
    "Olival tradicional em encosta suave (oliveiras em grade de 7 m, solo pedregoso)"
  ]
  title "olive_grove"
  contactProperties [
    ContactProperties {{
      material1 "soil"
      coulombFriction [
        {fmt(SOIL_FRICTION)}
      ]
    }}
    # Roda sobre pedra gera mais de 10 pontos de contato; com o limite
    # padrão (10) o Webots avisa no console a cada pedra.
    ContactProperties {{
      maxContactJoints 50
    }}
  ]
}}
Viewpoint {{
  orientation -0.17 0.24 0.96 1.0
  position -62.0 -50.0 24.0
}}
TexturedBackground {{
  texture "noon_cloudy_countryside"
}}
TexturedBackgroundLight {{
  texture "noon_cloudy_countryside"
}}
"""
    trees, positions = tree_nodes(terrain, rng)
    parts = [header, outer_ground_node(terrain), terrain_node(terrain),
             road_node(terrain)]
    parts.extend(trees)
    parts.extend(rock_nodes(terrain, rng, positions))
    parts.extend(obstacle_nodes(terrain, rng, positions))
    parts.extend(vineyard_nodes(terrain, rng))
    parts.extend(tank_nodes(terrain))
    parts.append(f"""DEF AGROBOT agrobot {{
  translation {fmt(ROBOT_X)} {fmt(ROBOT_Y)} {fmt(robot_z)}
  name "agrobot"
  controller "<extern>"
}}
""")
    return ''.join(parts)


def main():
    """Lê os argumentos e escreve o .wbt."""
    here = os.path.dirname(os.path.abspath(__file__))
    default_output = os.path.join(here, '..', 'worlds', 'olive_grove.wbt')
    parser = argparse.ArgumentParser(
        description=__doc__.strip().splitlines()[0])
    parser.add_argument('--seed', type=int, default=11,
                        help='semente do sorteio (padrão 11)')
    parser.add_argument('--slope', type=float, default=SLOPE,
                        help='declive máximo da encosta, de sul para norte '
                             '(padrão 0.05 = 5%%; 0 deixa o chão plano)')
    parser.add_argument('--noise', type=float, default=1.0,
                        help='escala do relevo irregular: ruído, montículos '
                             'e covas (padrão 1; 0 deixa só a encosta e a '
                             'ondulação larga)')
    parser.add_argument('--output', default=default_output,
                        help='arquivo .wbt de saída')
    args = parser.parse_args()
    if not 0.0 <= args.slope <= 0.08:
        parser.error('--slope precisa estar entre 0 e 0.08')
    if args.noise < 0.0:
        parser.error('--noise precisa ser >= 0')
    text = world_text(args.seed, args.slope, args.noise)
    with open(args.output, 'w', encoding='utf-8') as world:
        world.write(text)
    print(f'{os.path.normpath(args.output)}: {len(text) // 1024} KiB')


if __name__ == '__main__':
    main()
