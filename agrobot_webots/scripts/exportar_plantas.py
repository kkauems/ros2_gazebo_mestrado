#!/usr/bin/env python3
"""
Inventário das plantas de um mundo do Webots, lido direto do .wbt.

Lê cada CropPlant (plantation_field.wbt) e cada OliveTree (olive_grove.wbt)
e escreve uma linha por planta: nome, fileira, posição da base no mundo,
altura e os raios usados na avaliação da aquisição. Ler o .wbt (e não
regenerar o mundo pelo gerador) garante que o inventário é o do mundo que
rodou. Também lê a pose inicial do robô (DEF AGROBOT), que a avaliação usa
quando a pasta de fotos não tem o origem.yaml.

    python3 agrobot_webots/scripts/exportar_plantas.py plantation_field
    python3 agrobot_webots/scripts/exportar_plantas.py olive_grove -o plantas.csv

Colunas do CSV:
    planta, tipo, fileira, indice   nome no .wbt ("crop plant r2 14": fileira
                                    r2, índice 14 na fileira)
    x, y, z                         base da planta no mundo (m)
    yaw                             rotação em torno de z (rad)
    altura                          do chão ao topo (m): colmo + pendão no
                                    milho, topo da copa na oliveira
    raio_base                       raio do colmo / tronco (m)
    raio_largo, z_largo             meia largura da planta vista de lado e a
                                    altura (acima da base) onde ela é mais
                                    larga: 0 no milho (as folhas não contam
                                    para "planta inteira"), copa na oliveira
"""

import argparse
import csv
import math
import os
import re
import sys

WORLDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'worlds')

# Padrões dos PROTOs (protos/CropPlant.proto e protos/OliveTree.proto), para
# campos que o .wbt omite.
DEFAULTS = {
    'CropPlant': {'height': 1.4, 'stalkRadius': 0.02, 'leafLength': 0.55},
    'OliveTree': {'height': 3.4, 'trunkHeight': 1.1, 'trunkRadius': 0.14,
                  'crownRadius': 1.4},
}
# Pendão do CropPlant: cone de 0,16 m centrado 0,08 m acima do colmo.
CROP_TASSEL = 0.16

FIELDS = ['planta', 'tipo', 'fileira', 'indice', 'x', 'y', 'z', 'yaw',
          'altura', 'raio_base', 'raio_largo', 'z_largo']


def world_path(world):
    """Caminho do .wbt a partir do nome (plantation_field) ou de um caminho."""
    if os.path.isfile(world):
        return world
    name = world if world.endswith('.wbt') else world + '.wbt'
    path = os.path.normpath(os.path.join(WORLDS_DIR, name))
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Mundo não encontrado: {path}')
    return path


def top_level_nodes(text, proto):
    """Corpo ({...}) de cada nó proto do .wbt (sem nós aninhados)."""
    bodies = []
    for match in re.finditer(r'(?:DEF\s+\S+\s+)?\b' + proto + r'\s*\{', text):
        depth = 0
        start = match.end() - 1
        for i in range(start, len(text)):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
                if depth == 0:
                    bodies.append(text[start + 1:i])
                    break
    return bodies


def field(body, name, count=1):
    """Valor de um campo do nó: float, tupla de floats ou string."""
    match = re.search(r'^\s*' + name + r'\s+(.+)$', body, re.MULTILINE)
    if match is None:
        return None
    value = match.group(1).strip()
    if value.startswith('"'):
        return value.strip('"')
    numbers = [float(v) for v in value.split()[:count]]
    return numbers[0] if count == 1 else tuple(numbers)


def rotation_yaw(rotation):
    """Yaw de um SFRotation que gira só em torno de z (o caso dos mundos)."""
    if rotation is None:
        return 0.0
    ax, ay, az, angle = rotation
    if abs(ax) > 1e-6 or abs(ay) > 1e-6:
        raise ValueError(f'rotação fora do eixo z: {rotation}')
    return math.atan2(math.sin(angle * az), math.cos(angle * az))


def plant_row(name):
    match = re.search(r'\br(\d+)\s+(\d+)$', name or '')
    if match is None:
        return '', ''
    return f'r{match.group(1)}', int(match.group(2))


def read_plants(world):
    """Lista de dicionários (colunas FIELDS), uma por planta do mundo."""
    with open(world_path(world)) as wbt:
        text = wbt.read()
    plants = []
    for proto, kind in (('CropPlant', 'milho'), ('OliveTree', 'oliveira')):
        defaults = DEFAULTS[proto]
        for body in top_level_nodes(text, proto):
            def value(key):
                v = field(body, key)
                return defaults[key] if v is None else v
            x, y, z = field(body, 'translation', 3) or (0.0, 0.0, 0.0)
            name = field(body, 'name') or f'{proto} {len(plants)}'
            row, index = plant_row(name)
            plant = {'planta': name, 'tipo': kind, 'fileira': row,
                     'indice': index, 'x': x, 'y': y, 'z': z,
                     'yaw': rotation_yaw(field(body, 'rotation', 4))}
            if proto == 'CropPlant':
                plant.update(altura=value('height') + CROP_TASSEL,
                             raio_base=value('stalkRadius'),
                             raio_largo=0.0, z_largo=0.0)
            else:
                trunk = value('trunkHeight')
                height = max(value('height'), trunk + 1.0)
                plant.update(altura=height, raio_base=value('trunkRadius'),
                             raio_largo=value('crownRadius'),
                             # meio da copa (ela começa 0,25 m acima do tronco)
                             z_largo=0.5 * (trunk + 0.25 + height))
            plants.append(plant)
    return plants


def read_robot_start(world):
    """(x, y, z, yaw) do robô no .wbt (DEF AGROBOT), ou None."""
    with open(world_path(world)) as wbt:
        text = wbt.read()
    bodies = top_level_nodes(text, 'agrobot')
    if not bodies:
        return None
    x, y, z = field(bodies[0], 'translation', 3) or (0.0, 0.0, 0.0)
    return x, y, z, rotation_yaw(field(bodies[0], 'rotation', 4))


def write_csv(plants, out):
    writer = csv.DictWriter(out, fieldnames=FIELDS)
    writer.writeheader()
    for plant in plants:
        writer.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v)
                         for k, v in plant.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('mundo', help='plantation_field, olive_grove ou um .wbt')
    parser.add_argument('-o', '--saida', help='CSV de saída (padrão: tela)')
    args = parser.parse_args()
    plants = read_plants(args.mundo)
    if not plants:
        sys.exit(f'Nenhuma CropPlant ou OliveTree em {args.mundo}.')
    if args.saida:
        with open(args.saida, 'w', newline='') as out:
            write_csv(plants, out)
        rows = len({p['fileira'] for p in plants})
        print(f'{len(plants)} plantas em {rows} fileiras -> {args.saida}')
    else:
        write_csv(plants, sys.stdout)


if __name__ == '__main__':
    main()
