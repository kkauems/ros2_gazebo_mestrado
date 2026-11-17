"""
Controlador Webots da pessoa dos testes da camada de segurança.

Move um Pedestrian (PROTO do Webots) por uma trajetória, com a mesma
animação de passos do controlador "pedestrian" do Webots (Apache 2.0,
Cyberbotics), e grava a posição dele num CSV a cada passo, para medir a
distância até o robô. Usado pelos mundos que o scripts/teste_pessoa.py gera;
não entra nos mundos do trabalho.

Argumentos (controllerArgs):
    --trajectory "x1 y1, x2 y2, ..."   pontos no frame do mundo (um ponto só:
                                       a pessoa fica parada nele)
    --speed V                          velocidade de caminhada (m/s)
    --start T                          tempo de simulação em que começa a
                                       andar (antes fica parada no 1º ponto)
    --ground "z1, z2, ..."             altura do chão em cada ponto
    --start-file arquivo               começa a andar quando o arquivo existir
                                       (o teste cria quando o robô chega perto)
    --once                             anda a trajetória uma vez e para no fim
                                       (sem isso, vai e volta sem parar)
    --log arquivo.csv                  t, x, y, andando (0/1)
"""

import math
import optparse
import os

ROOT_HEIGHT = 1.27
CYCLE_TO_DISTANCE_RATIO = 0.22
WALK_SEQUENCES_NUMBER = 8
JOINT_NAMES = [
    'leftArmAngle', 'leftLowerArmAngle', 'leftHandAngle',
    'rightArmAngle', 'rightLowerArmAngle', 'rightHandAngle',
    'leftLegAngle', 'leftLowerLegAngle', 'leftFootAngle',
    'rightLegAngle', 'rightLowerLegAngle', 'rightFootAngle',
    'headAngle',
]
HEIGHT_OFFSETS = [-0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03]
ANGLES = [
    [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],
    [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],
    [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],
    [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],
    [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],
    [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],
    [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],
    [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],
    [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],
    [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],
    [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],
    [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],
    [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09],
]


def parse_points(text):
    return [[float(v) for v in p.split()] for p in text.split(',') if p.strip()]


def position_on_path(points, ground, distance, once=False):
    """Ponto (x, y, z, rumo) a uma distância do início, em ida e volta."""
    if len(points) == 1:
        return points[0][0], points[0][1], ground[0], math.pi
    if once:
        path, heights = list(points), list(ground)
        lengths = [math.dist(path[i], path[i + 1])
                   for i in range(len(path) - 1)]
        distance = min(distance, sum(lengths))
        path.append(path[-1])
        heights.append(heights[-1])
        lengths.append(0.0)
        if distance >= sum(lengths):
            a, b = points[-2], points[-1]
            return (b[0], b[1], ground[-1],
                    math.atan2(b[1] - a[1], b[0] - a[0]))
    else:
        path = points + points[-2:0:-1]
        heights = ground + ground[-2:0:-1]
        lengths = [math.dist(path[i], path[(i + 1) % len(path)])
                   for i in range(len(path))]
        distance %= sum(lengths)
    for i, length in enumerate(lengths):
        if distance <= length or i == len(lengths) - 1:
            a, b = path[i], path[(i + 1) % len(path)]
            ratio = distance / length if length > 0 else 0.0
            z = heights[i] + ratio * (heights[(i + 1) % len(path)] - heights[i])
            return (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]),
                    z, math.atan2(b[1] - a[1], b[0] - a[0]))
        distance -= length


def main():
    parser = optparse.OptionParser()
    parser.add_option('--trajectory', default='')
    parser.add_option('--speed', type=float, default=1.0)
    parser.add_option('--start', type=float, default=0.0)
    parser.add_option('--ground', default='')
    parser.add_option('--log', default='')
    parser.add_option('--start-file', dest='start_file', default='')
    parser.add_option('--once', action='store_true', default=False)
    options, _ = parser.parse_args()
    points = parse_points(options.trajectory)
    ground = [float(v) for v in options.ground.split(',')] if options.ground \
        else [0.0] * len(points)

    # Só aqui: o módulo do Webots não existe fora dele (testes).
    from controller import Supervisor
    robot = Supervisor()
    step = int(robot.getBasicTimeStep())
    node = robot.getSelf()
    translation = node.getField('translation')
    rotation = node.getField('rotation')
    joints = [node.getField(name) for name in JOINT_NAMES]
    log = open(options.log, 'w') if options.log else None
    if log:
        log.write('t,x,y,andando\n')

    started = None
    total = sum(math.dist(points[i], points[i + 1])
                for i in range(len(points) - 1))
    while robot.step(step) != -1:
        now = robot.getTime()
        if started is None and now >= options.start and (
                not options.start_file or os.path.exists(options.start_file)):
            started = now
        walked = (now - started) * options.speed if started is not None \
            else 0.0
        walking = (started is not None and len(points) > 1
                   and not (options.once and walked >= total))
        x, y, z, heading = position_on_path(points, ground, walked,
                                            options.once)
        offset = 0.0
        if walking:
            phase = walked / CYCLE_TO_DISTANCE_RATIO
            sequence = int(phase) % WALK_SEQUENCES_NUMBER
            ratio = phase - int(phase)
            following = (sequence + 1) % WALK_SEQUENCES_NUMBER
            for field, angles in zip(joints, ANGLES):
                field.setSFFloat(angles[sequence] * (1 - ratio)
                                 + angles[following] * ratio)
            offset = (HEIGHT_OFFSETS[sequence] * (1 - ratio)
                      + HEIGHT_OFFSETS[following] * ratio)
        translation.setSFVec3f([x, y, z + ROOT_HEIGHT + offset])
        rotation.setSFRotation([0, 0, 1, heading])
        if log:
            log.write(f'{now:.3f},{x:.4f},{y:.4f},{int(walking)}\n')
            log.flush()


if __name__ == '__main__':
    main()
