#!/usr/bin/env python3
r"""
Teste da camada de segurança: uma pessoa no caminho do robô.

Põe um Pedestrian do Webots num cenário da plantação ou do olival (parada
na entrelinha ou atravessando na frente do robô), comanda o robô pelo Nav2 ou
como operador (/cmd_vel_teleop, com prioridade sobre o Nav2) e mede, pela
pose verdadeira, a menor distância entre a pessoa e o robô e o que o
collision_monitor fez.

Uso (três terminais, ou o primeiro em segundo plano):

    # 1. gera o mundo do cenário em install/.../worlds/<mundo>_pessoa.wbt
    python3 agrobot_webots/scripts/teste_pessoa.py mundo plantacao_parada
    # 2. simulação (EKF + VSLAM, como sempre; teleop:=keyboard sobe o mux)
    ros2 launch agrobot_webots simulation.launch.py \
        world:=plantation_field_pessoa.wbt ekf:=true vslam:=true \
        vslam_db:=teste_pessoa teleop:=keyboard
    # 3. a pessoa (controlador extern do Webots) e a medição
    python3 agrobot_webots/scripts/teste_pessoa.py rodar plantacao_parada

Os resultados vão para dados/seguranca/: <cenario>_<rotulo>_<data>.csv (pose
do robô e da pessoa e a ação do collision_monitor a 10 Hz) e uma linha por
rodada em resumo.md. --rotulo distingue rodadas (por exemplo sem-seguranca,
com safety:=false no launch, como controle).
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

# Medidas no frame do mundo do Webots. Os dois mundos começam com o robô
# olhando para +x, então odom = mundo - partida.
WORLDS = {
    'plantation_field': {'start': (-6.0, 0.0), 'terrain': 'FIELD_TERRAIN'},
    'olive_grove': {'start': (-36.0, -3.5), 'terrain': 'GROVE_TERRAIN'},
}

# Cada cenário: mundo, trajetória da pessoa no odom (um ponto: parada),
# velocidade, se ela começa a andar quando o robô chega a uma distância
# (gatilho, m entre o centro do robô e o 1º ponto) e o comando do robô: um
# objetivo do Nav2 (x, y no frame map/odom da partida) ou o operador a uma
# velocidade fixa para a frente. 'duracao' é o limite em s de simulação.
SCENARIOS = {
    # Pessoa parada no meio da entrelinha y = 0; o Nav2 tenta passar.
    'plantacao_parada': {
        'world': 'plantation_field', 'person': [(5.0, 0.0)],
        'goal': (9.5, 0.0), 'duracao': 120.0,
    },
    # Pessoa atravessa a entrelinha y = 0 a 1 m/s, saindo de dentro da
    # fileira, quando o robô está a 2 m: chega ao meio da entrelinha com ela
    # ~0,6 m à frente do para-choque (o robô a 0,5 m/s).
    'plantacao_cruzando': {
        'world': 'plantation_field',
        'person': [(4.5, -1.6), (4.5, 1.6)], 'speed': 1.0, 'trigger': 2.0,
        'goal': (9.5, 0.0), 'duracao': 120.0,
    },
    # O operador manda o robô a 0,4 m/s para cima da pessoa parada.
    'plantacao_operador': {
        'world': 'plantation_field', 'person': [(5.0, 0.0)],
        'teleop': 0.4, 'duracao': 40.0,
    },
    # Olival: pessoa parada na entrelinha larga (7 m); o Nav2 contorna. O
    # objetivo fica a 11 m: o costmap global só aceita até ~12 m.
    'olival_parada': {
        'world': 'olive_grove', 'person': [(8.0, 0.0)],
        'goal': (11.0, 0.0), 'duracao': 150.0,
    },
    # Olival: pessoa atravessa a entrelinha na frente do robô.
    'olival_cruzando': {
        'world': 'olive_grove',
        'person': [(8.0, -2.5), (8.0, 2.5)], 'speed': 1.0, 'trigger': 2.4,
        'goal': (11.0, 0.0), 'duracao': 150.0,
    },
}

# Pegada do robô (base_link) e raio da pessoa (tronco e pernas).
HALF_LENGTH = 0.6
HALF_WIDTH = 0.56
PERSON_RADIUS = 0.25
ACTIONS = {0: 'nada', 1: 'parada', 2: 'lentidao', 3: 'aproximacao',
           4: 'limite'}


def share_dir():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('agrobot_webots')


def project_dir():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.dirname(os.path.dirname(here))


def terrain(text, name):
    """Altura do chão (função x, y no mundo) pela ElevationGrid do mundo."""
    solid = text.index(f'DEF {name} Solid')
    tx, ty, tz = (float(v) for v in re.search(
        r'translation\s+(\S+)\s+(\S+)\s+(\S+)', text[solid:]).groups())
    grid = text[solid:]
    nx = int(re.search(r'xDimension\s+(\d+)', grid).group(1))
    dx = float(re.search(r'xSpacing\s+(\S+)', grid).group(1))
    ny = int(re.search(r'yDimension\s+(\d+)', grid).group(1))
    dy = float(re.search(r'ySpacing\s+(\S+)', grid).group(1))
    start = grid.index('height [') + len('height [')
    values = [float(v) for v in grid[start:grid.index(']', start)].split()]

    def height(x, y):
        u = min(max((x - tx) / dx, 0.0), nx - 1.001)
        v = min(max((y - ty) / dy, 0.0), ny - 1.001)
        i, j = int(u), int(v)
        fu, fv = u - i, v - j

        def h(a, b):
            return values[a + b * nx]
        return tz + ((1 - fu) * (1 - fv) * h(i, j) + fu * (1 - fv) * h(i + 1, j)
                     + (1 - fu) * fv * h(i, j + 1) + fu * fv * h(i + 1, j + 1))
    return height


def make_world(name):
    scenario = SCENARIOS[name]
    world = WORLDS[scenario['world']]
    worlds = os.path.join(share_dir(), 'worlds')
    with open(os.path.join(worlds, scenario['world'] + '.wbt')) as f:
        text = f.read()
    sx, sy = world['start']
    x, y = scenario['person'][0]
    z = terrain(text, world['terrain'])(x + sx, y + sy)
    proto = ('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/'
             'webots/R2025a/projects/humans/pedestrian/protos/'
             'Pedestrian.proto"\n')
    header_end = text.index('\n', text.index('#VRML_SIM')) + 1
    text = text[:header_end] + proto + text[header_end:]
    # A pessoa é um Robot com controlador extern: o rodar sobe o controlador
    # (pessoa_teste.py) com a trajetória do cenário.
    text += f"""DEF PESSOA Pedestrian {{
  translation {x + sx:.3f} {y + sy:.3f} {z + 1.27:.3f}
  name "pessoa"
  controller "<extern>"
  controllerArgs ["--teste"]
  shirtColor 0.9 0.4 0.1
}}
"""
    out = os.path.join(worlds, f'{scenario["world"]}_pessoa.wbt')
    if os.path.islink(out):
        os.remove(out)
    with open(out, 'w') as f:
        f.write(text)
    print(f'Mundo gerado: {out} (pessoa em {x:.2f}, {y:.2f} do odom).')
    print(f'Rode: ros2 launch agrobot_webots simulation.launch.py '
          f'world:={scenario["world"]}_pessoa.wbt ekf:=true vslam:=true '
          f'vslam_db:=teste_pessoa teleop:=keyboard')


def clearance(robot_x, robot_y, yaw, person_x, person_y):
    """Distância entre o círculo da pessoa e o retângulo do robô."""
    dx, dy = person_x - robot_x, person_y - robot_y
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    ox = max(abs(lx) - HALF_LENGTH, 0.0)
    oy = max(abs(ly) - HALF_WIDTH, 0.0)
    return math.hypot(ox, oy) - PERSON_RADIUS


def run(name, label, output_dir):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry

    scenario = SCENARIOS[name]
    sx, sy = WORLDS[scenario['world']]['start']
    os.makedirs(output_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d-%H%M%S')
    prefix = os.path.join(output_dir, f'{name}_{label}_{stamp}')
    person_log = prefix + '_pessoa.csv'
    start_file = prefix + '_iniciar'

    # A pessoa: controlador extern no Webots que já está rodando.
    text = open(os.path.join(share_dir(), 'worlds',
                             scenario['world'] + '.wbt')).read()
    height = terrain(text, WORLDS[scenario['world']]['terrain'])
    points = [(x + sx, y + sy) for x, y in scenario['person']]
    controller = os.path.join(share_dir(), 'controllers', 'pessoa_teste',
                              'pessoa_teste.py')
    webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
    args = [os.path.join(webots_home, 'webots-controller'),
            '--robot-name=pessoa', controller,
            '--trajectory=' + ', '.join(f'{x:.3f} {y:.3f}' for x, y in points),
            '--ground=' + ', '.join(f'{height(x, y):.3f}' for x, y in points),
            f'--speed={scenario.get("speed", 1.0)}', '--once',
            f'--start-file={start_file}', f'--log={person_log}']
    person = subprocess.Popen(args, stdout=open(prefix + '_pessoa.log', 'w'),
                              stderr=subprocess.STDOUT)

    class Test(Node):
        def __init__(self):
            super().__init__('teste_pessoa', parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True)])
            self.robot = None
            self.action = (0, '')
            self.actions = {}
            self.rows = []
            self.min_clearance = math.inf
            self.t0 = None
            self.goal_result = None
            self.started = False
            self.create_subscription(Odometry, '/ground_truth/odom',
                                     self.on_robot, 10)
            self.create_subscription(CollisionMonitorState,
                                     '/collision_monitor_state',
                                     self.on_state, 10)
            self.teleop = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
            self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        def on_robot(self, message):
            p = message.pose.pose
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self.robot = (p.position.x, p.position.y, yaw,
                          message.twist.twist.linear.x)

        def on_state(self, message):
            self.action = (message.action_type, message.polygon_name)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

    rclpy.init()
    node = Test()
    print('Esperando a pose verdadeira e o Nav2...')
    while rclpy.ok() and node.robot is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    if 'goal' in scenario:
        node.nav.wait_for_server()
    # O Nav2 e o RTAB-Map precisam de alguns segundos depois de subir.
    t_ready = node.now() + 5.0
    while rclpy.ok() and node.now() < t_ready:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.t0 = node.now()
    goal_future = None
    if 'goal' in scenario:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = scenario['goal']
        goal.pose.pose.orientation.w = 1.0
        goal_future = node.nav.send_goal_async(goal)
    print(f'Cenário {name} começou.')
    result_future = None
    last_row = -1.0
    person_xy = points[0]
    person_seen = 0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        t = node.now() - node.t0
        if t > scenario['duracao']:
            break
        if 'teleop' in scenario:
            twist = Twist()
            twist.linear.x = scenario['teleop']
            node.teleop.publish(twist)
        if goal_future is not None and goal_future.done() and \
                result_future is None:
            result_future = goal_future.result().get_result_async()
        if result_future is not None and result_future.done():
            node.goal_result = result_future.result().status
            break
        # Posição atual da pessoa (última linha do log do controlador).
        try:
            with open(person_log) as f:
                lines = f.read().splitlines()
            if len(lines) > person_seen + 1:
                person_seen = len(lines) - 1
                _, px, py, _ = lines[-1].split(',')
                person_xy = (float(px), float(py))
        except (OSError, ValueError):
            pass
        rx, ry, yaw, speed = node.robot
        gap = clearance(rx, ry, yaw, *person_xy)
        node.min_clearance = min(node.min_clearance, gap)
        # Gatilho: o centro do robô a 'trigger' m (ao longo de x) da linha
        # que a pessoa atravessa.
        trigger = scenario.get('trigger')
        if trigger and not node.started and points[0][0] - rx < trigger:
            open(start_file, 'w').close()
            node.started = True
            print(f't = {t:.1f} s: a pessoa começou a atravessar.')
        action, polygon = node.action
        if action:
            node.actions.setdefault((ACTIONS.get(action, action), polygon), t)
        if t - last_row >= 0.1:
            last_row = t
            node.rows.append((t, rx - sx, ry - sy, yaw, speed,
                              person_xy[0] - sx, person_xy[1] - sy, gap,
                              ACTIONS.get(action, action), polygon))

    person.terminate()
    if os.path.exists(start_file):
        os.remove(start_file)
    with open(prefix + '.csv', 'w') as f:
        f.write('t_s,robo_x,robo_y,robo_yaw,robo_v,pessoa_x,pessoa_y,'
                'folga_m,acao,zona\n')
        for row in node.rows:
            f.write(','.join(f'{v:.3f}' if isinstance(v, float) else str(v)
                             for v in row) + '\n')
    rx, ry = node.robot[0] - sx, node.robot[1] - sy
    statuses = {4: 'chegou', 5: 'cancelado', 6: 'abortado'}
    result = ('-' if 'goal' not in scenario
              else statuses.get(node.goal_result, 'sem resposta no tempo'))
    first = '; '.join(f'{a} ({z}) em {t:.1f} s'
                      for (a, z), t in sorted(node.actions.items(),
                                              key=lambda i: i[1]))
    contact = 'SIM' if node.min_clearance <= 0.0 else 'não'
    line = (f'| {name} | {label} | {node.min_clearance:.2f} | {contact} | '
            f'{result} | ({rx:.1f}; {ry:.1f}) | {first or "nenhuma"} |')
    summary = os.path.join(output_dir, 'resumo.md')
    if not os.path.exists(summary):
        with open(summary, 'w') as f:
            f.write('| Cenário | Rótulo | Menor folga (m) | Contato | Nav2 | '
                    'Pose final (odom) | Primeira ação de cada zona |\n'
                    '|---|---|---|---|---|---|---|\n')
    with open(summary, 'a') as f:
        f.write(line + '\n')
    print(line)
    print(f'CSV: {prefix}.csv')
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('comando', choices=['mundo', 'rodar'])
    parser.add_argument('cenario', choices=sorted(SCENARIOS))
    parser.add_argument('--rotulo', default='com-seguranca')
    parser.add_argument('--saida', default=os.path.join(
        project_dir(), 'dados', 'seguranca'))
    args = parser.parse_args()
    if args.comando == 'mundo':
        make_world(args.cenario)
    else:
        run(args.cenario, args.rotulo, args.saida)


if __name__ == '__main__':
    sys.exit(main())
