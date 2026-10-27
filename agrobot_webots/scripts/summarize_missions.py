#!/usr/bin/env python3
"""
Resume as missões de waypoints gravadas pelo waypoint_mission.

Lê os pares <caso>_<modo>_<data>_waypoints.csv / _trajetoria.csv e imprime
uma tabela em Markdown, uma linha por rodada:

    python3 agrobot_webots/scripts/summarize_missions.py ~/.ros/agrobot_missions

Colunas:
- atingidos: waypoints com status "atingido" / total;
- tempo: tempo de simulação até o fim da missão (s);
- distância: caminho percorrido de verdade (pose do Supervisor, m);
- erro final: distância verdadeira entre o robô e o último waypoint (m);
- ATE: raiz do erro quadrático médio da posição estimada contra a
  verdadeira ao longo da trajetória (m), sem alinhar as trajetórias (as duas
  começam no mesmo frame, o da partida);
- erro máx.: maior erro de localização na trajetória (m);
- yaw máx.: maior erro de orientação na trajetória (graus).
"""

import argparse
import csv
import glob
import math
import os


def read_rows(path):
    with open(path, newline='') as csv_file:
        return list(csv.DictReader(csv_file))


def number(value):
    return float(value) if value not in (None, '') else None


def summarize(waypoints_path):
    waypoints = read_rows(waypoints_path)
    name = os.path.basename(waypoints_path)[:-len('_waypoints.csv')]
    # <caso>_<modo>_<AAAAMMDD-HHMMSS>
    case_and_mode, stamp = name.rsplit('_', 1)
    case, mode = case_and_mode.rsplit('_', 1)
    reached = sum(1 for row in waypoints if row['status'] == 'atingido')
    last = waypoints[-1]
    errors = []
    yaw_errors = []
    trajectory_path = waypoints_path.replace('_waypoints.csv',
                                             '_trajetoria.csv')
    if os.path.exists(trajectory_path):
        for row in read_rows(trajectory_path):
            values = [number(row[k]) for k in
                      ('est_x', 'est_y', 'real_x', 'real_y')]
            if None not in values:
                ex, ey, rx, ry = values
                errors.append(math.hypot(ex - rx, ey - ry))
            yaws = [number(row[k]) for k in ('est_yaw', 'real_yaw')]
            if None not in yaws:
                diff = yaws[0] - yaws[1]
                yaw_errors.append(abs(math.degrees(
                    math.atan2(math.sin(diff), math.cos(diff)))))
    ate = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors \
        else None
    return {
        'caso': case,
        'modo': mode,
        'data': stamp,
        'atingidos': f'{reached}/{len(waypoints)}',
        'tempo': number(last['t_saida_s']),
        'distância': number(last['distancia_real_m']),
        'erro final': number(last['erro_real_alvo_m']),
        'ATE': ate,
        'erro máx.': max(errors) if errors else None,
        'yaw máx.': max(yaw_errors) if yaw_errors else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument(
        'paths', nargs='*',
        default=[os.path.expanduser('~/.ros/agrobot_missions')],
        help='pastas ou arquivos *_waypoints.csv')
    args = parser.parse_args()
    files = []
    for path in args.paths:
        if os.path.isdir(path):
            files += glob.glob(os.path.join(path, '*_waypoints.csv'))
        else:
            files.append(path)
    rows = sorted((summarize(f) for f in files),
                  key=lambda r: (r['caso'], r['modo'], r['data']))
    columns = ['caso', 'modo', 'data', 'atingidos', 'tempo', 'distância',
               'erro final', 'ATE', 'erro máx.', 'yaw máx.']
    print('| ' + ' | '.join(columns) + ' |')
    print('|' + '---|' * len(columns))
    for row in rows:
        cells = []
        for column in columns:
            value = row[column]
            if isinstance(value, float):
                digits = 0 if column == 'tempo' else \
                    1 if column == 'yaw máx.' else 2
                value = f'{value:.{digits}f}'
            cells.append('' if value is None else str(value))
        print('| ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
