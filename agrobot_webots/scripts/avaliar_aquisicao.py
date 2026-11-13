#!/usr/bin/env python3
"""
Avalia as fotos gravadas pelo data_recorder: cobertura das plantas e erro de
georreferência.

Para cada pasta de fotos (fotos.csv + camera_info.yaml + origem.yaml), lê o
inventário das plantas do .wbt (exportar_plantas.py) e projeta cada planta
em cada foto, com a pose verdadeira (real_*) e com a estimada (est_*):

    python3 agrobot_webots/scripts/avaliar_aquisicao.py dados/fotos/<pasta>
    python3 agrobot_webots/scripts/avaliar_aquisicao.py dados/fotos/*/ --folha 12

Cobertura (pela pose verdadeira, que é onde a foto foi feita de fato):
- planta inteira numa foto: o pé (--corte-base metros acima da base), o
  topo e, na oliveira, as duas bordas da copa caem dentro da imagem (com
  --margem pixels de folga), na frente da câmera e a menos de --dist-max
  metros (na horizontal). O --dist-max descarta as fileiras de trás, que a
  fileira da frente tapa (padrão: 1,8 m na plantação, só a fileira vizinha;
  7 m no olival). As folhas do milho não contam: "inteira" é o colmo até o
  pendão. O --corte-base (padrão 0,25 m) existe porque a câmera, a ~0,9 m do
  chão e a 0,9 m da fileira, não vê os ~20 cm de baixo do colmo (conferido
  nas imagens do Webots); com 0, quase nenhuma planta de milho sai inteira;
- planta vista: o meio da planta (metade da altura) cai dentro da imagem;
- cobertura >= 1 e >= 3: fração das plantas do mundo inteiras em pelo menos
  1 e 3 fotos. Com uma câmera só, que vê um lado por passada, a cobertura do
  mundo inteiro é baixa por construção; o plantas.csv diz quais fileiras a
  rota cobriu.

Erro de georreferência (só nas fotos com pose estimada):
- câmera: distância horizontal (m) e diferença de yaw (graus) entre a pose
  estimada e a verdadeira, como no fotos.csv;
- planta (m): onde a planta vai parar no mapa se a foto for georreferenciada
  pela pose estimada (ponto da planta no frame da câmera verdadeira, levado
  ao mapa pela estimada), contra onde ela está; soma o erro de posição e o
  de yaw vezes a distância;
- pixels: distância, na imagem, entre o meio da planta projetado com a pose
  estimada e com a verdadeira, para cada planta inteira na foto.

Saída: uma tabela em Markdown na tela (uma linha por pasta) e, em
<pasta>/avaliacao/: resumo.md, plantas.csv (uma linha por planta, com as
fotos em que aparece), projecoes.csv (uma linha por planta inteira em cada
foto) e, com --folha N, folha_contato.png com N fotos e as plantas
projetadas (verde: pose verdadeira, magenta: estimada; pede matplotlib).

O frame das poses é o da partida (o odom e o map nascem na pose inicial do
robô). O origem.yaml que o data_recorder grava diz onde ele fica no mundo;
sem ele (fotos antigas), a pose do robô no .wbt é usada e o z pode ficar
alguns centímetros errado.
"""

import argparse
import csv
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from exportar_plantas import read_plants, read_robot_start  # noqa: E402

DIST_MAX = {'milho': 1.8, 'oliveira': 7.0}
NEAR = 0.05   # plano de recorte da DataCamera (m)
WORLD_HINTS = (('plantation_field', 'plantation_field'),
               ('olive_grove', 'olive_grove'),
               ('plantacao', 'plantation_field'),
               ('olival', 'olive_grove'))


# Entradas --------------------------------------------------------------------

def read_camera_info(path):
    with open(path) as yaml:
        text = yaml.read()
    width = int(re.search(r'image_width:\s*(\d+)', text).group(1))
    height = int(re.search(r'image_height:\s*(\d+)', text).group(1))
    data = re.search(r'camera_matrix:.*?data:\s*\[([^\]]*)\]', text, re.S)
    k = [float(v) for v in data.group(1).split(',')]
    return {'width': width, 'height': height, 'fx': k[0], 'fy': k[4],
            'cx': k[2], 'cy': k[5]}


def read_origin(folder, world):
    """(posição no mundo, yaw) da partida e de onde veio."""
    path = os.path.join(folder, 'origem.yaml')
    if os.path.exists(path):
        values = {}
        with open(path) as yaml:
            for line in yaml:
                key, _, value = line.partition(':')
                if value.strip() and not key.startswith('#'):
                    values[key.strip()] = float(value)
        return (np.array([values['x'], values['y'], values['z']]),
                values['yaw'], 'origem.yaml')
    start = read_robot_start(world)
    if start is None:
        raise RuntimeError(f'{folder}: sem origem.yaml e sem robô no .wbt')
    return np.array(start[:3]), start[3], 'pose do robô no .wbt'


def guess_world(folder):
    name = os.path.basename(os.path.normpath(folder))
    for hint, world in WORLD_HINTS:
        if hint in name:
            return world
    return None


def number(value):
    return float(value) if value not in (None, '') else None


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose(row, prefix):
    """(posição, rotação do frame óptico) de uma linha do fotos.csv."""
    keys = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    values = [number(row.get(f'{prefix}_{k}')) for k in keys]
    if None in values:
        return None
    return np.array(values[:3]), quaternion_to_matrix(*values[3:])


def read_photos(folder):
    with open(os.path.join(folder, 'fotos.csv'), newline='') as csv_file:
        rows = list(csv.DictReader(csv_file))
    photos = []
    for row in rows:
        photos.append({'foto': int(row['foto']), 'arquivo': row['arquivo'],
                       'real': pose(row, 'real'), 'est': pose(row, 'est'),
                       'erro_xy_m': number(row.get('erro_xy_m')),
                       'erro_yaw_rad': number(row.get('erro_yaw_rad'))})
    return photos


# Geometria -------------------------------------------------------------------

def plants_in_start_frame(plants, origin, yaw0):
    """Base de cada planta no frame da partida (o das colunas real_*)."""
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    back = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    for plant in plants:
        world = np.array([plant['x'], plant['y'], plant['z']])
        plant['base'] = back @ (world - origin)
    return plants


def key_points(plant, camera_position, base_cut=0.0):
    """
    Pontos que precisam estar na imagem para a planta estar inteira: pé
    (base_cut acima da base), topo e, se ela tem largura (copa), as duas
    bordas vistas da câmera (perpendiculares ao raio horizontal câmera ->
    planta). Na copa, o topo vai meio raio para o lado da câmera: a parte de
    cima da copa mais perto dela é a que sai primeiro pelo alto da imagem.
    """
    base = plant['base']
    up = np.array([0.0, 0.0, 1.0])
    top = base + plant['altura'] * up
    points = [base + min(base_cut, 0.5 * plant['altura']) * up]
    radius = plant['raio_largo']
    if radius > 0.0:
        ray = np.append(base[:2] - camera_position[:2], 0.0)
        norm = np.linalg.norm(ray)
        ray = ray / norm if norm > 1e-6 else np.array([0.0, 1.0, 0.0])
        side = np.array([-ray[1], ray[0], 0.0])
        wide = base + plant['z_largo'] * up
        points += [wide + radius * side, wide - radius * side]
        top = top - 0.5 * radius * ray
    return points + [top]


def middle_point(plant):
    return plant['base'] + np.array([0.0, 0.0, 0.5 * plant['altura']])


def project(point, camera, info):
    """(u, v) do ponto na imagem, ou None se estiver atrás da câmera."""
    position, rotation = camera
    p = rotation.T @ (point - position)   # no frame óptico: z para a frente
    if p[2] < NEAR:
        return None
    return (info['fx'] * p[0] / p[2] + info['cx'],
            info['fy'] * p[1] / p[2] + info['cy'])


def inside(uv, info, margin):
    return (uv is not None
            and margin <= uv[0] <= info['width'] - 1 - margin
            and margin <= uv[1] <= info['height'] - 1 - margin)


def georeference_error(point, real, est):
    """Erro horizontal (m) do ponto georreferenciado pela pose estimada."""
    in_camera = real[1].T @ (point - real[0])
    placed = est[1] @ in_camera + est[0]
    return float(np.linalg.norm((placed - point)[:2]))


# Avaliação -------------------------------------------------------------------

def evaluate(folder, world, dist_max, margin, base_cut=0.25):
    info = read_camera_info(os.path.join(folder, 'camera_info.yaml'))
    origin, yaw0, origin_source = read_origin(folder, world)
    plants = plants_in_start_frame(read_plants(world), origin, yaw0)
    photos = read_photos(folder)
    for plant in plants:
        plant['fotos_inteira'] = []
        plant['fotos_vista'] = []

    projections = []
    for photo in photos:
        real, est = photo['real'], photo['est']
        photo['inteiras'] = 0
        if real is None:
            continue
        for plant in plants:
            limit = dist_max or DIST_MAX[plant['tipo']]
            if np.linalg.norm(plant['base'][:2] - real[0][:2]) > limit:
                continue
            middle = middle_point(plant)
            uv_real = project(middle, real, info)
            if inside(uv_real, info, 0):
                plant['fotos_vista'].append(photo['foto'])
            whole = all(inside(project(p, real, info), info, margin)
                        for p in key_points(plant, real[0], base_cut))
            if not whole:
                continue
            plant['fotos_inteira'].append(photo['foto'])
            photo['inteiras'] += 1
            row = {'foto': photo['foto'], 'arquivo': photo['arquivo'],
                   'planta': plant['planta'], 'fileira': plant['fileira'],
                   'u_real': uv_real[0], 'v_real': uv_real[1],
                   'u_est': '', 'v_est': '', 'erro_px': '', 'erro_m': '',
                   'base_real': project(plant['base'], real, info),
                   'topo_real': project(plant['base'] + np.array(
                       [0.0, 0.0, plant['altura']]), real, info)}
            if est is not None:
                row['erro_m'] = georeference_error(middle, real, est)
                uv_est = project(middle, est, info)
                if uv_est is not None:
                    row.update(u_est=uv_est[0], v_est=uv_est[1],
                               erro_px=math.hypot(uv_est[0] - uv_real[0],
                                                  uv_est[1] - uv_real[1]))
                row['base_est'] = project(plant['base'], est, info)
                row['topo_est'] = project(plant['base'] + np.array(
                    [0.0, 0.0, plant['altura']]), est, info)
            projections.append(row)

    return {'pasta': folder, 'mundo': world, 'info': info,
            'origem': origin_source, 'plantas': plants, 'fotos': photos,
            'projecoes': projections}


def stats(values):
    values = [v for v in values if v not in (None, '')]
    if not values:
        return None
    a = np.abs(np.array(values, dtype=float))
    return {'media': float(a.mean()), 'p95': float(np.percentile(a, 95)),
            'max': float(a.max()), 'n': len(a)}


def summarize(result):
    plants = result['plantas']
    photos = result['fotos']
    total = len(plants)
    once = sum(1 for p in plants if len(p['fotos_inteira']) >= 1)
    thrice = sum(1 for p in plants if len(p['fotos_inteira']) >= 3)
    seen = sum(1 for p in plants if p['fotos_vista'])
    name = os.path.basename(os.path.normpath(result['pasta']))
    parts = name.rsplit('_', 2)
    return {
        'pasta': name,
        'modo': parts[1] if len(parts) == 3 else '',
        'fotos': len(photos),
        'fotos_est': sum(1 for p in photos if p['est'] is not None),
        'plantas': total,
        'cobertura_1': once, 'cobertura_3': thrice, 'vistas': seen,
        'cam_xy': stats([p['erro_xy_m'] for p in photos]),
        'cam_yaw': stats([None if p['erro_yaw_rad'] is None
                          else math.degrees(p['erro_yaw_rad'])
                          for p in photos]),
        'planta_m': stats([r['erro_m'] for r in result['projecoes']]),
        'planta_px': stats([r['erro_px'] for r in result['projecoes']]),
        'fileiras': rows_coverage(plants),
    }


def rows_coverage(plants):
    rows = {}
    for plant in plants:
        r = rows.setdefault(plant['fileira'], [0, 0, 0])
        r[0] += 1
        r[1] += len(plant['fotos_inteira']) >= 1
        r[2] += len(plant['fotos_inteira']) >= 3
    return rows


def fmt(stat, key='media', digits=2):
    return '—' if stat is None else f'{stat[key]:.{digits}f}'


def percent(count, total):
    return f'{count} ({100.0 * count / total:.0f}%)' if total else '0'


HEADER = ('| pasta | modo | fotos | inteiras ≥1 | inteiras ≥3 | vistas ≥1 '
          '| câmera xy (m) média/máx | yaw (°) média/máx '
          '| planta (m) média/p95 | planta (px) média/p95 |\n'
          '|---|---|---|---|---|---|---|---|---|---|')


def table_row(s):
    n = s['plantas']
    return (f"| {s['pasta']} | {s['modo']} | {s['fotos']} "
            f"| {percent(s['cobertura_1'], n)} | {percent(s['cobertura_3'], n)} "
            f"| {percent(s['vistas'], n)} "
            f"| {fmt(s['cam_xy'])} / {fmt(s['cam_xy'], 'max')} "
            f"| {fmt(s['cam_yaw'], digits=1)} / "
            f"{fmt(s['cam_yaw'], 'max', 1)} "
            f"| {fmt(s['planta_m'])} / {fmt(s['planta_m'], 'p95')} "
            f"| {fmt(s['planta_px'], digits=0)} / "
            f"{fmt(s['planta_px'], 'p95', 0)} |")


# Saída -----------------------------------------------------------------------

def write_outputs(result, summary, args):
    out_dir = os.path.join(result['pasta'], 'avaliacao')
    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(out_dir, 'plantas.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['planta', 'fileira', 'indice', 'x', 'y',
                         'fotos_inteira', 'fotos_vista', 'lista_inteira'])
        for p in result['plantas']:
            writer.writerow([p['planta'], p['fileira'], p['indice'],
                             f"{p['x']:.3f}", f"{p['y']:.3f}",
                             len(p['fotos_inteira']), len(p['fotos_vista']),
                             ' '.join(map(str, p['fotos_inteira']))])

    fields = ['foto', 'arquivo', 'planta', 'fileira', 'u_real', 'v_real',
              'u_est', 'v_est', 'erro_px', 'erro_m']
    with open(os.path.join(out_dir, 'projecoes.csv'), 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for row in result['projecoes']:
            writer.writerow({k: (f'{v:.3f}' if isinstance(v, float) else v)
                             for k, v in row.items() if k in fields})

    rows = summary['fileiras']
    lines = [
        f"# Avaliação da aquisição: {summary['pasta']}", '',
        f"Mundo `{result['mundo']}`, {summary['plantas']} plantas; "
        f"{summary['fotos']} fotos ({summary['fotos_est']} com pose "
        f"estimada). Partida: {result['origem']}. "
        f"Distância máxima: {args.dist_max or 'padrão'}; corte da base "
        f"{args.corte_base} m; margem {args.margem} px.", '',
        HEADER, table_row(summary), '',
        '| fileira | plantas | inteiras ≥1 | inteiras ≥3 |', '|---|---|---|---|']
    for name in sorted(rows):
        total, once, thrice = rows[name]
        lines.append(f'| {name} | {total} | {percent(once, total)} '
                     f'| {percent(thrice, total)} |')
    with open(os.path.join(out_dir, 'resumo.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    if args.folha:
        contact_sheet(result, args.folha,
                      os.path.join(out_dir, 'folha_contato.png'))


def contact_sheet(result, count, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.image as mpimg
        import matplotlib.pyplot as plt
    except ImportError:
        print('  folha de contato: sem matplotlib '
              '(sudo apt install python3-matplotlib)', file=sys.stderr)
        return
    candidates = [p for p in result['fotos'] if p['inteiras'] > 0]
    if not candidates:
        print('  folha de contato: nenhuma foto com planta inteira',
              file=sys.stderr)
        return
    picks = [candidates[round(i * (len(candidates) - 1) / max(count - 1, 1))]
             for i in range(min(count, len(candidates)))]
    picks = list({p['foto']: p for p in picks}.values())
    by_photo = {}
    for row in result['projecoes']:
        by_photo.setdefault(row['foto'], []).append(row)
    cols = min(4, len(picks))
    lines = math.ceil(len(picks) / cols)
    info = result['info']
    fig, axes = plt.subplots(lines, cols, figsize=(4 * cols, 3.2 * lines),
                             squeeze=False)
    for ax in axes.flat:
        ax.axis('off')
    for ax, photo in zip(axes.flat, picks):
        image = mpimg.imread(os.path.join(result['pasta'], photo['arquivo']))
        ax.imshow(image[::2, ::2], extent=(0, info['width'],
                                           info['height'], 0))
        errors = []
        for row in by_photo.get(photo['foto'], []):
            for key, color in (('real', 'lime'), ('est', 'magenta')):
                base, top = row.get(f'base_{key}'), row.get(f'topo_{key}')
                if base is not None and top is not None:
                    ax.plot([base[0], top[0]], [base[1], top[1]], color=color,
                            linewidth=1.2)
            if row['erro_px'] != '':
                errors.append(row['erro_px'])
        title = f"foto {photo['foto']}"
        if errors:
            title += f' · erro médio {np.mean(errors):.0f} px'
        ax.set_title(title, fontsize=8)
        ax.set_xlim(0, info['width'])
        ax.set_ylim(info['height'], 0)
    fig.suptitle(f"{os.path.basename(os.path.normpath(result['pasta']))} "
                 '(verde: pose verdadeira; magenta: estimada)', fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('pastas', nargs='+',
                        help='pastas de fotos (com fotos.csv)')
    parser.add_argument('--mundo', help='plantation_field, olive_grove ou um '
                        '.wbt (padrão: pelo nome da pasta)')
    parser.add_argument('--dist-max', type=float, default=None,
                        help='distância horizontal máxima câmera-planta (m)')
    parser.add_argument('--margem', type=float, default=5.0,
                        help='folga da borda da imagem (px)')
    parser.add_argument('--corte-base', type=float, default=0.25,
                        help='altura acima da base a partir da qual a planta '
                        'precisa aparecer para contar como inteira (m)')
    parser.add_argument('--folha', type=int, default=0, metavar='N',
                        help='grava uma folha de contato com N fotos')
    args = parser.parse_args()

    folders = [f for f in args.pastas
               if os.path.exists(os.path.join(f, 'fotos.csv'))]
    if not folders:
        sys.exit('Nenhuma pasta com fotos.csv.')
    print(HEADER)
    for folder in folders:
        world = args.mundo or guess_world(folder)
        if world is None:
            sys.exit(f'{folder}: não deu para saber o mundo pelo nome; '
                     'use --mundo.')
        result = evaluate(folder, world, args.dist_max, args.margem,
                          args.corte_base)
        summary = summarize(result)
        write_outputs(result, summary, args)
        print(table_row(summary))
        if result['origem'] != 'origem.yaml':
            print(f'  ({folder}: sem origem.yaml; partida tirada do .wbt)',
                  file=sys.stderr)


if __name__ == '__main__':
    main()
