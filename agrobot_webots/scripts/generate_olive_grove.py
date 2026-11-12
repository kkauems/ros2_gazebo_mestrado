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
    python3 agrobot_webots/scripts/generate_olive_grove.py --slope 0.05

O que o mundo tem (x para leste, ao longo das fileiras; y para norte, encosta
acima; z para cima):
- terreno ElevationGrid de 120 x 100 m que sobe para norte (2 a 3% no
  olival), com ondulação larga e rugosidade fina;
- 70 lugares de oliveira (OliveTree) em grade de 7 x 7 m (10 por fileira,
  7 fileiras), com falhas e árvores novas;
- lascas de pedra soltas no chão (o robô passa por cima), pedras pequenas e
  blocos na linha das oliveiras (o LiDAR vê os blocos);
- vinha ao norte (4 fileiras com postes), estrada de terra ao sul e um
  tanque de rega com uma casinha a leste;
- o Agrobot na cabeceira oeste, de frente para a entrelinha y = -3,5.
"""

import argparse
import math
import os
import random

# Terreno: grade de 120 x 100 m centrada na origem.
GRID_X = 120.0
GRID_Y = 100.0
GRID_SPACING = 0.5
EDGE_FADE = 12.0        # na borda o relevo vai a 0 (encontra o chão de fora)

# Relevo (m). A encosta é uma rampa suave (smoothstep) de sul para norte, com
# declive máximo SLOPE no meio. O LiDAR é um plano só, a ~13 cm do chão, e o
# Nav2 marca obstáculo até 5 m: com curvatura k, o chão 5 m à frente se afasta
# k * 5² / 2 do plano do robô. Aqui k fica abaixo de 0,003 1/m (< 4 cm).
SLOPE = 0.03
UNDULATION = 0.15
UNDULATION_WAVELENGTH = (50.0, 100.0)
ROUGHNESS = 0.003
ROUGHNESS_WAVELENGTH = (2.5, 6.0)

# Olival: grade de 7 x 7 m, fileiras ao longo de x.
TREE_SPACING = 7.0
TREES_PER_ROW = 10      # x = -31,5 a 31,5
ROW_COUNT = 7           # y = -21 a 21
MISSING_TREES = 0.05
YOUNG_TREES = 0.12
FIELD_HALF_X = 38.0     # borda do olival (cabeceiras de ~6 m)
FIELD_HALF_Y = 25.0

# Pedras.
CHIPS = 220             # lascas chatas (2 a 4 cm), em todo o olival
STONES = 40             # pedras pequenas (topo 5 a 6,5 cm), na linha das árvores
BIG_ROCKS = 5           # blocos (topo 15 a 23 cm), na linha das árvores

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
    """Altura do chão h(x, y): encosta, ondulação e rugosidade."""

    def __init__(self, rng, slope=SLOPE):
        """Sorteia as ondas."""
        # Altura total da encosta: smoothstep em GRID_Y tem declive máximo
        # 1,5 * altura / GRID_Y.
        self.rise = slope * GRID_Y / 1.5
        self.waves = []
        for _ in range(3):
            self.waves.append(self._wave(
                rng, *UNDULATION_WAVELENGTH, UNDULATION / 2.0))
        for _ in range(10):
            self.waves.append(self._wave(
                rng, *ROUGHNESS_WAVELENGTH, ROUGHNESS / 2.0))
        # Desloca para cima, para o relevo não ficar negativo.
        self.offset = UNDULATION + ROUGHNESS

    @staticmethod
    def _wave(rng, min_wavelength, max_wavelength, amplitude):
        wavelength = rng.uniform(min_wavelength, max_wavelength)
        direction = rng.uniform(0.0, math.pi)
        k = 2.0 * math.pi / wavelength
        return (k * math.cos(direction), k * math.sin(direction),
                rng.uniform(0.0, 2.0 * math.pi),
                amplitude * rng.uniform(0.6, 1.0))

    def height(self, x, y):
        """Altura do chão (m) no ponto (x, y) do mundo."""
        height = self.offset + self.rise * smoothstep(
            -0.5 * GRID_Y, 0.5 * GRID_Y, y)
        for kx, ky, phase, amplitude in self.waves:
            height += amplitude * math.sin(kx * x + ky * y + phase)
        edge = min(0.5 * GRID_X - abs(x), 0.5 * GRID_Y - abs(y))
        return smoothstep(0.0, EDGE_FADE, edge) * height


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
    """Lascas soltas, pedras pequenas e blocos."""
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
    # Blocos na linha das árvores, entre duas oliveiras (o LiDAR vê).
    columns = tree_columns()
    for k in range(BIG_ROCKS):
        x = rng.choice(columns[1:-1]) + 0.5 * TREE_SPACING
        y = rng.choice(rows) + rng.uniform(-0.3, 0.3)
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


def world_text(seed, slope=SLOPE):
    """Conteúdo completo do .wbt."""
    rng = random.Random(seed)
    terrain = Terrain(rng, slope)
    robot_z = terrain.height(ROBOT_X, ROBOT_Y) + ROBOT_HEIGHT
    header = f"""#VRML_SIM R2025a utf8
# Olival tradicional do planalto (Douro Internacional) para o Agrobot.
# GERADO por agrobot_webots/scripts/generate_olive_grove.py
# (seed {seed}, slope {fmt(slope)}). Não edite à mão: mude o script e gere
# de novo.

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
  orientation -0.25 0.2 0.95 1.8
  position -6.0 -62.0 26.0
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
        type "prickly"
        colorOverride 0.68 0.62 0.45
        textureTransform TextureTransform {{
          scale 80 80
        }}
      }}
      geometry Plane {{
        size 300 300
      }}
    }}
  ]
  name "outer_ground"
  boundingObject Plane {{
    size 300 300
  }}
  locked TRUE
}}
"""
    trees, positions = tree_nodes(terrain, rng)
    parts = [header, terrain_node(terrain), road_node(terrain)]
    parts.extend(trees)
    parts.extend(rock_nodes(terrain, rng, positions))
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
                             '(padrão 0.03 = 3%%; 0 deixa o chão plano)')
    parser.add_argument('--output', default=default_output,
                        help='arquivo .wbt de saída')
    args = parser.parse_args()
    if not 0.0 <= args.slope <= 0.08:
        parser.error('--slope precisa estar entre 0 e 0.08')
    text = world_text(args.seed, args.slope)
    with open(args.output, 'w', encoding='utf-8') as world:
        world.write(text)
    print(f'{os.path.normpath(args.output)}: {len(text) // 1024} KiB')


if __name__ == '__main__':
    main()
