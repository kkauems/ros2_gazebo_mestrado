#!/usr/bin/env python3
"""
Estatística de uma bateria de rodadas (bateria.py): média e desvio padrão
por caso de estudo.

    python3 agrobot_webots/scripts/estatistica_bateria.py dados/bateria/<data>
    python3 agrobot_webots/scripts/estatistica_bateria.py <pasta> --por-rodada

Para cada caso (todas as rodadas juntas), média ± desvio padrão amostral de:
- ATE: RMSE da posição estimada contra a verdadeira (pose do Supervisor) ao
  longo da trajetória, no frame da partida, sem alinhar (m);
- erro máx.: maior erro de posição de cada rodada (m);
- yaw RMSE: RMSE do erro de orientação (graus);
- atingidos: fração dos waypoints que o Nav2 deu como atingidos (%);
- no alvo: fração dos waypoints em que o robô esteve de verdade a até
  0,5 m do alvo ao sair dele (%);
- paradas: quantas vezes o robô ficou parado (velocidade verdadeira
  < STOP_SPEED e sem girar) por pelo menos STOP_MIN_S fora da parada
  programada nos waypoints (estimativa a mais de AT_WAYPOINT do alvo
  atual): as hesitações do Nav2 e as paradas da camada de segurança;
- tempo: tempo de simulação da missão (s).
"""

import argparse
import glob
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from summarize_missions import number, read_rows, summarize  # noqa: E402

STOP_SPEED = 0.05      # m/s
STOP_TURN = 0.15       # rad/s
STOP_MIN_S = 1.0       # s
AT_WAYPOINT = 0.5      # m
START_IGNORE_S = 3.0   # s (arrancada)
TRUE_LIMIT = 0.5       # m


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def trajectory_metrics(waypoints_path):
    """Paradas fora dos waypoints e RMSE do yaw."""
    targets = {int(r['indice']): (float(r['alvo_x']), float(r['alvo_y']))
               for r in read_rows(waypoints_path)}
    rows = []
    for row in read_rows(waypoints_path.replace('_waypoints.csv',
                                                '_trajetoria.csv')):
        values = [number(row[k]) for k in ('t_s', 'est_x', 'est_y', 'est_yaw',
                                          'real_x', 'real_y', 'real_yaw')]
        if None not in values:
            rows.append((values, int(row['waypoint_atual'])))
    yaw_sq = [math.degrees(wrap(v[3] - v[6])) ** 2 for v, _ in rows]
    yaw_rmse = math.sqrt(sum(yaw_sq) / len(yaw_sq)) if yaw_sq else None
    stops = 0
    stopped_since = None
    counted = False
    # Velocidade pela pose verdadeira numa janela de ~1 s (5 amostras).
    for i in range(5, len(rows)):
        (t, ex, ey, _, rx, ry, ryaw), wp = rows[i]
        (t0, _, _, _, rx0, ry0, ryaw0), _ = rows[i - 5]
        dt = t - t0
        if dt <= 0:
            continue
        speed = math.hypot(rx - rx0, ry - ry0) / dt
        turn = abs(wrap(ryaw - ryaw0)) / dt
        target = targets.get(wp)
        near = target is not None and \
            math.hypot(ex - target[0], ey - target[1]) <= AT_WAYPOINT
        still = speed < STOP_SPEED and turn < STOP_TURN and not near \
            and t > START_IGNORE_S
        if still:
            if stopped_since is None:
                stopped_since, counted = t, False
            if not counted and t - stopped_since >= STOP_MIN_S:
                stops += 1
                counted = True
        else:
            stopped_since = None
    return stops, yaw_rmse


def run_metrics(path):
    row = summarize(path)
    got, total = (int(v) for v in row['atingidos'].split('/'))
    hits = 0
    for wp in read_rows(path):
        error = number(wp.get('erro_real_alvo_m'))
        hits += error is not None and error <= TRUE_LIMIT
    stops, yaw_rmse = trajectory_metrics(path)
    row.update(
        atingidos_pct=100.0 * got / total, no_alvo_pct=100.0 * hits / total,
        completa=got == total, paradas=stops, yaw_rmse=yaw_rmse,
        n_wp=total, got=got, hits=hits)
    return row


def fmt(value, digits=2):
    return '' if value is None else f'{value:.{digits}f}'.replace('.', ',')


def mean_sd(values, digits=2):
    values = [v for v in values if v is not None]
    if not values:
        return ''
    text = fmt(statistics.mean(values), digits)
    if len(values) > 1:
        text += ' ± ' + fmt(statistics.stdev(values), digits)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('paths', nargs='+',
                        help='pastas da bateria ou arquivos *_waypoints.csv')
    parser.add_argument('--por-rodada', action='store_true',
                        help='também a tabela com uma linha por rodada')
    args = parser.parse_args()
    files = []
    for path in args.paths:
        if os.path.isdir(path):
            files += glob.glob(os.path.join(path, '*_waypoints.csv'))
        else:
            files.append(path)
    groups = {}
    for path in sorted(files):
        row = run_metrics(path)
        groups.setdefault((row['caso'], row['modo']), []).append(row)

    columns = ['caso', 'modo', 'rodadas', 'ATE (m)', 'erro máx. (m)',
               'yaw RMSE (°)', 'atingidos (%)', 'no alvo ≤ 0,5 m (%)',
               'rodadas completas', 'paradas', 'tempo (s)']
    print('| ' + ' | '.join(columns) + ' |')
    print('|' + '---|' * len(columns))
    for (case, mode), runs in sorted(groups.items()):
        print('| ' + ' | '.join([
            case, mode, str(len(runs)),
            mean_sd([r['ATE'] for r in runs]),
            mean_sd([r['erro máx.'] for r in runs]),
            mean_sd([r['yaw_rmse'] for r in runs], 1),
            mean_sd([r['atingidos_pct'] for r in runs], 0),
            mean_sd([r['no_alvo_pct'] for r in runs], 0),
            f'{sum(r["completa"] for r in runs)}/{len(runs)}',
            mean_sd([r['paradas'] for r in runs], 1),
            mean_sd([r['tempo'] for r in runs], 0)]) + ' |')
    print('\nMédia ± desvio padrão amostral entre as rodadas.')

    if args.por_rodada:
        columns = ['caso', 'data', 'atingidos', 'no alvo', 'ATE (m)',
                   'erro máx. (m)', 'yaw RMSE (°)', 'paradas', 'tempo (s)']
        print('\n| ' + ' | '.join(columns) + ' |')
        print('|' + '---|' * len(columns))
        for (case, mode), runs in sorted(groups.items()):
            for r in runs:
                print('| ' + ' | '.join([
                    case, r['data'], f'{r["got"]}/{r["n_wp"]}',
                    f'{r["hits"]}/{r["n_wp"]}', fmt(r['ATE']),
                    fmt(r['erro máx.']), fmt(r['yaw_rmse'], 1),
                    str(r['paradas']), fmt(r['tempo'], 0)]) + ' |')


if __name__ == '__main__':
    main()
