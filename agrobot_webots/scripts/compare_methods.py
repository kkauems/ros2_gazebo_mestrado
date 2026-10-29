#!/usr/bin/env python3
"""
Compara os métodos de localização em várias rodadas de cada caso.

Lê os mesmos CSVs do summarize_missions.py e imprime uma tabela em Markdown
com uma linha por caso e modo (todas as rodadas juntas), e depois a lista
dos waypoints perdidos:

    python3 agrobot_webots/scripts/compare_methods.py ~/.ros/agrobot_missions

Colunas:
- rodadas: quantas rodadas daquele caso e modo;
- atingidos: waypoints que o Nav2 deu como atingidos, somando as rodadas /
  total, e quantas rodadas chegaram a todos;
- no alvo: waypoints em que o robô esteve de verdade a até TRUE_LIMIT do alvo
  (pose verdadeira na saída do waypoint). Difere de "atingidos" quando a
  localização está errada: o Nav2 acha que chegou, mas o robô está em outro
  lugar (por exemplo, na entrelinha vizinha);
- ATE: média e desvio padrão (amostral) do ATE das rodadas (m);
- erro máx.: média do maior erro de localização de cada rodada e, entre
  parênteses, o maior de todas (m);
- yaw máx.: o maior erro de orientação de todas as rodadas (graus);
- tempo: tempo de simulação médio da missão (s).

Um waypoint perdido é contado como falha de localização quando o erro de
localização no momento em que o Nav2 desistiu dele passava de LOC_LIMIT
(metade da folga até a entrelinha vizinha); senão, como falha de navegação
(planejador ou controlador, com a localização certa).
"""

import argparse
import glob
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from summarize_missions import number, read_rows, summarize  # noqa: E402

LOC_LIMIT = 0.5
TRUE_LIMIT = 0.5


def on_target(waypoints_path):
    count = 0
    for row in read_rows(waypoints_path):
        error = number(row.get('erro_real_alvo_m'))
        count += error is not None and error <= TRUE_LIMIT
    return count


def lost_waypoints(waypoints_path):
    lost = []
    for row in read_rows(waypoints_path):
        if row['status'] == 'atingido':
            continue
        error = number(row.get('erro_localizacao_m'))
        kind = 'navegação' if error is not None and error <= LOC_LIMIT \
            else 'localização'
        lost.append((row['nome'], kind, error))
    return lost


def fmt(value, digits=2):
    return '' if value is None else f'{value:.{digits}f}'.replace('.', ',')


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

    groups = {}
    for path in sorted(files):
        row = summarize(path)
        row['perdidos'] = lost_waypoints(path)
        row['no alvo'] = on_target(path)
        groups.setdefault((row['caso'], row['modo']), []).append(row)

    columns = ['caso', 'modo', 'rodadas', 'atingidos', 'rodadas completas',
               f'no alvo (≤ {fmt(TRUE_LIMIT, 1)} m)',
               'ATE média ± dp (m)', 'erro máx. médio (maior) (m)',
               'yaw máx. (°)', 'tempo médio (s)']
    print('| ' + ' | '.join(columns) + ' |')
    print('|' + '---|' * len(columns))
    for (case, mode), runs in sorted(groups.items()):
        reached = total = complete = 0
        hits = sum(r['no alvo'] for r in runs)
        for run in runs:
            got, of = (int(v) for v in run['atingidos'].split('/'))
            reached += got
            total += of
            complete += got == of
        ates = [r['ATE'] for r in runs if r['ATE'] is not None]
        maxima = [r['erro máx.'] for r in runs if r['erro máx.'] is not None]
        yaws = [r['yaw máx.'] for r in runs if r['yaw máx.'] is not None]
        times = [r['tempo'] for r in runs if r['tempo'] is not None]
        ate = fmt(statistics.mean(ates)) if ates else ''
        if len(ates) > 1:
            ate += ' ± ' + fmt(statistics.stdev(ates))
        worst = (fmt(statistics.mean(maxima)) + f' ({fmt(max(maxima))})'
                 if maxima else '')
        print('| ' + ' | '.join([
            case, mode, str(len(runs)), f'{reached}/{total}',
            f'{complete}/{len(runs)}', f'{hits}/{total}', ate, worst,
            fmt(max(yaws), 1) if yaws else '',
            fmt(statistics.mean(times), 0) if times else '']) + ' |')

    print()
    print('Waypoints perdidos (erro de localização quando o Nav2 desistiu):')
    for (case, mode), runs in sorted(groups.items()):
        for run in runs:
            for name, kind, error in run['perdidos']:
                error_text = '?' if error is None else fmt(error) + ' m'
                print(f'- {case} {mode} {run["data"]}: {name} '
                      f'({kind}, {error_text})')


if __name__ == '__main__':
    main()
