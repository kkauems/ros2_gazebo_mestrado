#!/usr/bin/env python3
"""
Bateria de avaliação: roda os casos de estudo várias vezes, uma rodada por
semente de ruído, com a configuração oficial (EKF + VSLAM mapeando).

    python3 agrobot_webots/scripts/bateria.py                       # casos 2 a 5, 5 sementes
    python3 agrobot_webots/scripts/bateria.py --cases caso2_plantacao_ida_volta --seeds 101 102
    python3 agrobot_webots/scripts/bateria.py --headless             # Webots sem janela (xvfb-run)

Cada rodada é um `ros2 launch agrobot_webots simulation.launch.py` com
`ekf:=true vslam:=true rviz:=false waypoints:=<caso> noise_seed:=<semente>`;
o script espera a missão terminar (linha "Missão ... terminada" no log),
fecha o launch e passa à próxima. A ordem é por semente: todas os casos da
semente 1, depois os da semente 2..., para que uma bateria interrompida
ainda tenha rodadas de todos os casos.

Grava em dados/bateria/<data>/: os CSVs das missões, o log de cada rodada e
rodadas.csv (caso, semente, resultado, CSV). Depois:

    python3 agrobot_webots/scripts/estatistica_bateria.py dados/bateria/<data>
"""

import argparse
import csv
import datetime
import glob
import os
import re
import shutil
import signal
import subprocess
import sys
import time

CASES = {
    'caso2_plantacao_ida_volta': 'plantation_field.wbt',
    'caso3_plantacao_serpentina': 'plantation_field.wbt',
    'caso4_olival_ida_volta': 'olive_grove.wbt',
    'caso5_olival_encosta': 'olive_grove.wbt',
    # Rodada de calibração das rodas (com wheel_calibration:=true
    # wheel_radius_scale:=1.0 lateral_correction:=none; seção 5 do manual).
    'calibracao_rodas_olival': 'olive_grove.wbt',
}
SEEDS = [101, 202, 303, 404, 505]

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MISSIONS = os.path.join(ROOT, 'dados', 'missions')
VSLAM = os.path.join(ROOT, 'dados', 'vslam')

END = re.compile(r'Missão (\S+ terminada:|cancelada|.*não iniciada)')
LEFTOVERS = ['nav2_', 'webots', 'rtabmap', 'collision_monitor',
             'robot_state_publisher', 'ekf_node', 'waypoint_mission',
             'stable_map', 'lidar_merge', 'wheel_slip', 'agrobot_',
             'ros2 launch', 'Xvfb']


def clean_leftovers():
    # Mata o que sobrou da rodada anterior, menos este script (o caminho dele
    # também tem 'agrobot_webots').
    me = {os.getpid(), os.getppid()}
    for pattern in LEFTOVERS:
        found = subprocess.run(['pgrep', '-f', pattern], capture_output=True,
                               text=True).stdout.split()
        for pid in map(int, found):
            if pid not in me:
                try:
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
    time.sleep(3)


def run_one(case, world, seed, out_dir, args):
    tag = f'{case}_semente{seed}'
    log_path = os.path.join(out_dir, 'logs', tag + '.log')
    db = f'bateria_{tag}'
    cmd = ['ros2', 'launch', 'agrobot_webots', 'simulation.launch.py',
           f'world:={world}', 'ekf:=true', 'vslam:=true', 'rviz:=false',
           f'waypoints:={case}', f'noise_seed:={seed}', f'vslam_db:={db}']
    cmd += args.extra
    if args.headless:
        cmd = ['xvfb-run', '-a', '-s', '-screen 0 1280x1024x24'] + cmd
    before = set(glob.glob(os.path.join(MISSIONS, '*_waypoints.csv')))
    start = time.time()
    result = 'tempo esgotado'
    with open(log_path, 'w') as log:
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        while True:
            with open(log_path, errors='replace') as f:
                text = f.read()
            match = END.search(text)
            if match and 'terminada' in match.group(0) and \
                    'CSV' not in text[match.start():]:
                # A linha "terminada" sai junto com os CSVs; espera ela.
                match = None
            if match:
                result = match.group(1).split(':')[0].strip()
                time.sleep(5)
                break
            if process.poll() is not None:
                result = f'launch saiu ({process.returncode})'
                break
            if time.time() - start > args.timeout * 60:
                break
            time.sleep(5)
        # Fecha o launch como um Ctrl-C e espera o rtabmap salvar.
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=180)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    clean_leftovers()
    seed_line = re.search(r'semente[^0-9]*(\d+)', open(log_path).read())
    new = sorted(set(glob.glob(os.path.join(MISSIONS, '*_waypoints.csv')))
                 - before)
    names = []
    for path in new:
        for suffix in ('_waypoints.csv', '_trajetoria.csv'):
            source = path.replace('_waypoints.csv', suffix)
            if os.path.exists(source):
                shutil.copy2(source, out_dir)
        names.append(os.path.basename(path))
    # Os bancos do mapeamento ocupam centenas de MB e não entram na conta.
    if not args.keep_db:
        for path in glob.glob(os.path.join(VSLAM, db + '*.db')):
            os.remove(path)
    return {
        'caso': case, 'semente': seed, 'resultado': result,
        'semente_log': seed_line.group(1) if seed_line else '',
        'minutos': f'{(time.time() - start) / 60:.1f}',
        'waypoints_csv': ' '.join(names),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--cases', nargs='+', default=[
        case for case in CASES if not case.startswith('calibracao')])
    parser.add_argument('--seeds', nargs='+', type=int, default=SEEDS)
    parser.add_argument('--out', default='')
    parser.add_argument('--headless', action='store_true',
                        help='Webots num display virtual (xvfb-run)')
    parser.add_argument('--timeout', type=float, default=150,
                        help='minutos por rodada antes de desistir')
    parser.add_argument('--keep-db', action='store_true')
    parser.add_argument('extra', nargs='*',
                        help='argumentos a mais para o launch (a:=b)')
    args = parser.parse_args()
    for case in args.cases:
        if case not in CASES:
            sys.exit(f'Caso desconhecido: {case}')
    out_dir = args.out or os.path.join(
        ROOT, 'dados', 'bateria',
        datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(os.path.join(out_dir, 'logs'), exist_ok=True)
    manifest = os.path.join(out_dir, 'rodadas.csv')
    done = set()
    if os.path.exists(manifest):
        with open(manifest, newline='') as f:
            done = {(r['caso'], int(r['semente'])) for r in csv.DictReader(f)}
    fields = ['caso', 'semente', 'resultado', 'semente_log', 'minutos',
              'waypoints_csv']
    for seed in args.seeds:
        for case in args.cases:
            if (case, seed) in done:
                continue
            print(f'[{datetime.datetime.now():%H:%M}] {case} semente {seed}',
                  flush=True)
            clean_leftovers()
            row = run_one(case, CASES[case], seed, out_dir, args)
            print(f'    {row["resultado"]} em {row["minutos"]} min '
                  f'({row["waypoints_csv"] or "sem CSV"})', flush=True)
            new_file = not os.path.exists(manifest)
            with open(manifest, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                if new_file:
                    writer.writeheader()
                writer.writerow(row)


if __name__ == '__main__':
    main()
