#!/usr/bin/env python3
"""
Gera os arquivos de dados do site (site/js/malha_chassi.js e
site/js/trajetorias.js).

O site abre direto do disco (file://), e o navegador não deixa uma página
assim ler outros arquivos com fetch. Por isso a malha do chassi e as
trajetórias vão embutidas em arquivos .js comuns.

Uso (na raiz do repositório):
    python3 site/gerar_dados.py [pasta_dos_resultados]

pasta_dos_resultados: onde estão os *_trajetoria.csv do passo 3
(resultados/passo3_comparacao/ dos arquivos do projeto). Sem ela, só a malha
é gerada.
"""

import base64
import csv
import json
import os
import sys

SITE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(SITE)
MESH = os.path.join(REPO, 'agrobot_webots', 'meshes', 'chassis.stl')

# Rodadas do caso 2 (plantação, ida e volta) mostradas no site: a de ATE
# mediano de cada método.
RUNS = {
    'ekf': ('caso2_plantacao_ida_volta_ekf_20261029-143152', 0.23),
    'ekf+vslam': ('caso2_plantacao_ida_volta_ekf+vslam_20261029-120833', 0.13),
    'ekf+slam': ('caso2_plantacao_ida_volta_ekf+slam_20261029-130903', 3.87),
}


def write_mesh():
    with open(MESH, 'rb') as f:
        data = base64.b64encode(f.read()).decode()
    with open(os.path.join(SITE, 'js', 'malha_chassi.js'), 'w') as f:
        f.write('// Gerado por site/gerar_dados.py a partir de '
                'agrobot_webots/meshes/chassis.stl. Não edite.\n')
        f.write(f'window.MALHA_CHASSI = "{data}";\n')


def write_trajectories(folder):
    out = {}
    for mode, (prefix, ate) in RUNS.items():
        path = os.path.join(folder, prefix + '_trajetoria.csv')
        rows = []
        with open(path) as f:
            for i, row in enumerate(csv.DictReader(f)):
                if i % 2:
                    continue
                values = [round(float(row[k]), 3) for k in (
                    't_s', 'est_x', 'est_y', 'est_yaw',
                    'real_x', 'real_y', 'real_yaw')]
                rows.append(values + [int(row['waypoint_atual'])])
        out[mode] = {'rodada': prefix, 'ate': ate, 'amostras': rows}
    with open(os.path.join(SITE, 'js', 'trajetorias.js'), 'w') as f:
        f.write('// Gerado por site/gerar_dados.py a partir dos CSVs do passo 3 '
                '(resultados/passo3_comparacao). Não edite.\n')
        f.write('// Colunas: t, est_x, est_y, est_yaw, real_x, real_y, '
                'real_yaw, waypoint_atual\n')
        f.write('window.TRAJETORIAS = ')
        json.dump(out, f, separators=(',', ':'))
        f.write(';\n')


if __name__ == '__main__':
    write_mesh()
    if len(sys.argv) > 1:
        write_trajectories(sys.argv[1])
