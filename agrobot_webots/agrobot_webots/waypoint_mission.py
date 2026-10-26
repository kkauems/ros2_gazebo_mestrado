#!/usr/bin/env python3
"""
Percorre uma lista de waypoints com o Nav2 e grava o resultado em CSV.

Lê os waypoints de um arquivo YAML (config/waypoints/) ou dos cliques no RViz
(ferramenta Publish Point, tópico /clicked_point), espera o Nav2 e o TF do
frame global até o base_link e manda tudo de uma vez para a ação
FollowWaypoints do waypoint_follower. O follower vai a um waypoint por vez
(NavigateToPose), para alguns segundos em cada um (WaitAtWaypoint, em
nav2_params.yaml) e, se um falhar, pula para o próximo.

As coordenadas do arquivo são do frame da partida do robô. O odom nasce na
pose inicial, e o map do slam_toolbox (e do RTAB-Map) também, então o mesmo
arquivo vale com e sem SLAM: o launch escolhe o frame (odom ou map).

Formato do arquivo:
    world: plantation_field.wbt      # opcional: recusa rodar em outro mundo
    waypoints:
      - {name: fim_entrelinha, x: 10.5, y: 0.0}
      - {name: cabeceira, x: 11.8, y: 1.8, yaw: 3.1416}
Sem yaw, o robô chega olhando para o próximo waypoint (o último, na direção
em que chegou).

Com arquivo, a missão começa sozinha. Sem arquivo (waypoints:=rviz), cada
clique do Publish Point acrescenta um waypoint e a missão espera os serviços:
    ~/start   inicia (ou repete) a missão com a lista atual
    ~/undo    tira o último waypoint
    ~/clear   apaga todos
    ~/save    grava a lista no formato acima (output_dir/rviz_<data>.yaml)

Saídas, em output_dir (padrão ~/.ros/agrobot_missions):
    <caso>_<modo>_<data>_waypoints.csv   uma linha por waypoint
    <caso>_<modo>_<data>_trajetoria.csv  pose estimada e verdadeira no tempo
A pose verdadeira vem de /ground_truth/odom (Supervisor do Webots), trazida
para o frame da partida. Ela só é usada para medir, nunca para navegar.

Tópicos:
    /waypoints/markers   (visualization_msgs/MarkerArray)  saída, para o RViz
    /ground_truth/odom   (nav_msgs/Odometry)               entrada, opcional
    /clicked_point       (geometry_msgs/PointStamped)      entrada, do RViz
"""

import csv
import math
import os
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

PENDING = 'pendente'
CURRENT = 'atual'
REACHED = 'atingido'
MISSED = 'perdido'

COLORS = {
    PENDING: (1.0, 0.65, 0.0),
    CURRENT: (0.1, 0.5, 1.0),
    REACHED: (0.1, 0.8, 0.2),
    MISSED: (0.9, 0.1, 0.1),
}

WAYPOINT_FIELDS = [
    'indice', 'nome', 'status', 'alvo_x', 'alvo_y', 'alvo_yaw',
    't_saida_s', 'est_x', 'est_y', 'est_yaw', 'real_x', 'real_y', 'real_yaw',
    'erro_est_alvo_m', 'erro_real_alvo_m', 'erro_localizacao_m',
    'erro_localizacao_yaw_rad', 'distancia_real_m',
]
TRAJECTORY_FIELDS = [
    't_s', 'waypoint_atual', 'est_x', 'est_y', 'est_yaw',
    'real_x', 'real_y', 'real_yaw',
]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def load_waypoints(path):
    """Lê o YAML; yaw_in é o yaw do arquivo (None se não tiver)."""
    with open(path) as waypoint_file:
        data = yaml.safe_load(waypoint_file) or {}
    entries = data.get('waypoints') or []
    if not entries:
        raise ValueError(f'nenhum waypoint em {path}')
    waypoints = []
    for i, entry in enumerate(entries):
        waypoints.append({
            'name': str(entry.get('name', f'wp{i + 1}')),
            'x': float(entry['x']),
            'y': float(entry['y']),
            'yaw_in': (None if entry.get('yaw') is None
                       else float(entry['yaw'])),
        })
    return data.get('world'), waypoints


def fill_yaw(waypoints, origin):
    """
    Completa wp['yaw'] a partir de wp['yaw_in'].

    Sem yaw: olhando para o próximo; o último, na direção em que chegou (o
    único, a partir de origin, a posição do robô).
    """
    for i, wp in enumerate(waypoints):
        if wp['yaw_in'] is not None:
            wp['yaw'] = wp['yaw_in']
        elif i + 1 < len(waypoints):
            nxt = waypoints[i + 1]
            wp['yaw'] = math.atan2(nxt['y'] - wp['y'], nxt['x'] - wp['x'])
        else:
            prev = waypoints[i - 1] if i > 0 else {
                'x': origin[0], 'y': origin[1]}
            wp['yaw'] = math.atan2(wp['y'] - prev['y'], wp['x'] - prev['x'])


class WaypointMission(Node):

    def __init__(self):
        super().__init__('waypoint_mission')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('run_label', '')
        self.declare_parameter('world', '')
        self.declare_parameter(
            'output_dir',
            os.path.join(os.path.expanduser('~'), '.ros', 'agrobot_missions'))
        self.declare_parameter('trajectory_rate', 5.0)

        path = self.get_parameter('waypoints_file').value
        self.frame = self.get_parameter('frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.run_label = self.get_parameter('run_label').value or self.frame
        self.output_dir = self.get_parameter('output_dir').value
        self.world = self.get_parameter('world').value
        if path:
            self.case = os.path.splitext(os.path.basename(path))[0]
            file_world, self.waypoints = load_waypoints(path)
        else:
            self.case = 'rviz'
            file_world, self.waypoints = None, []
        blocked = bool(file_world and self.world and file_world != self.world)
        if blocked:
            self.get_logger().error(
                f'{self.case} é para o mundo {file_world}, mas o mundo é '
                f'{self.world}. Missão cancelada.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/waypoints/markers', latched)
        self.create_subscription(
            Odometry, '/ground_truth/odom', self.ground_truth_callback, 10)
        self.truth_origin = None   # pose verdadeira na partida (mundo)
        self.truth = None          # pose verdadeira no frame da partida
        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_service(Trigger, '~/start', self.start_service)
        self.create_service(Trigger, '~/undo', self.undo_service)
        self.create_service(Trigger, '~/clear', self.clear_service)
        self.create_service(Trigger, '~/save', self.save_service)

        self.start_timer = None
        self.reset_progress()
        rate = float(self.get_parameter('trajectory_rate').value)
        self.create_timer(1.0 / rate, self.record_trajectory)
        if path and not blocked:
            self.start_mission()
        elif not path:
            self.get_logger().info(
                'Sem arquivo: clique no mapa com a ferramenta Publish Point '
                'do RViz para acrescentar waypoints e chame '
                '"ros2 service call /waypoint_mission/start '
                'std_srvs/srv/Trigger" para começar.')

    # Pose verdadeira -------------------------------------------------------

    def ground_truth_callback(self, message):
        p = message.pose.pose.position
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if self.truth_origin is None:
            self.truth_origin = (p.x, p.y, yaw)
        x0, y0, yaw0 = self.truth_origin
        dx, dy = p.x - x0, p.y - y0
        c, s = math.cos(yaw0), math.sin(yaw0)
        self.truth = (c * dx + s * dy, -s * dx + c * dy, wrap_angle(yaw - yaw0))
        if self.start_time is not None and not self.finished:
            if self.last_truth_xy is not None:
                self.truth_distance += math.hypot(
                    self.truth[0] - self.last_truth_xy[0],
                    self.truth[1] - self.last_truth_xy[1])
            self.last_truth_xy = self.truth[:2]

    def estimated_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame, self.robot_frame, Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(t.transform.rotation))

    # Edição pelo RViz ------------------------------------------------------

    def running(self):
        return self.start_timer is not None or (
            self.goal_handle is not None and not self.finished)

    def clicked_point_callback(self, message):
        if self.running():
            self.get_logger().warn('Missão em andamento: clique ignorado.')
            return
        x, y = message.point.x, message.point.y
        source = message.header.frame_id or self.frame
        if source != self.frame:
            try:
                t = self.tf_buffer.lookup_transform(self.frame, source, Time())
            except TransformException as error:
                self.get_logger().warn(f'Clique ignorado: {error}')
                return
            yaw = yaw_from_quaternion(t.transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
            x, y = (t.transform.translation.x + c * x - s * y,
                    t.transform.translation.y + s * x + c * y)
        n = len(self.waypoints) + 1
        self.waypoints.append(
            {'name': f'wp{n}', 'x': x, 'y': y, 'yaw_in': None})
        self.reset_progress()
        self.get_logger().info(
            f'Waypoint {n} acrescentado: ({x:.2f}, {y:.2f}) em {self.frame}')

    def start_service(self, request, response):
        response.success = self.start_mission()
        response.message = ('missão iniciada' if response.success else
                            'nada a fazer: lista vazia ou missão em andamento')
        return response

    def undo_service(self, request, response):
        response.success = bool(self.waypoints) and not self.running()
        if response.success:
            self.waypoints.pop()
            self.reset_progress()
        response.message = f'{len(self.waypoints)} waypoints'
        return response

    def clear_service(self, request, response):
        response.success = not self.running()
        if response.success:
            self.waypoints = []
            self.reset_progress()
        response.message = f'{len(self.waypoints)} waypoints'
        return response

    def save_service(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'lista vazia'
            return response
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(
            self.output_dir, f'rviz_{time.strftime("%Y%m%d-%H%M%S")}.yaml')
        lines = ['# Gravado pelo waypoint_mission (cliques no RViz).',
                 f'# Coordenadas no frame {self.frame}, a partir da partida.']
        if self.world:
            lines.append(f'world: {self.world}')
        lines.append('waypoints:')
        for wp in self.waypoints:
            yaw = '' if wp['yaw_in'] is None else f', yaw: {wp["yaw_in"]:.4f}'
            lines.append(f'  - {{name: {wp["name"]}, x: {wp["x"]:.3f}, '
                         f'y: {wp["y"]:.3f}{yaw}}}')
        with open(path, 'w') as out:
            out.write('\n'.join(lines) + '\n')
        response.success = True
        response.message = path
        self.get_logger().info(f'Waypoints gravados em {path}')
        return response

    # Envio da missão -------------------------------------------------------

    def reset_progress(self):
        """Volta todos os waypoints a pendente, para editar ou repetir."""
        self.status = [PENDING] * len(self.waypoints)
        self.records = [None] * len(self.waypoints)
        self.current = -1
        self.goal_handle = None
        self.sending = False
        self.finished = False
        self.start_time = None
        self.trajectory_rows = []
        self.truth_distance = 0.0
        self.last_truth_xy = None
        self.publish_markers()

    def start_mission(self):
        if self.running() or not self.waypoints:
            return False
        self.reset_progress()
        est = self.estimated_pose()
        fill_yaw(self.waypoints, est[:2] if est else (0.0, 0.0))
        self.get_logger().info(
            f'{self.case}: {len(self.waypoints)} waypoints no frame '
            f'{self.frame}. Esperando o Nav2 e o TF '
            f'{self.frame} -> {self.robot_frame}...')
        self.start_timer = self.create_timer(1.0, self.try_start)
        return True

    def try_start(self):
        if self.goal_handle is not None or self.sending:
            return
        if self.estimated_pose() is None:
            return
        if not self.client.server_is_ready():
            return
        goal = FollowWaypoints.Goal()
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.frame
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
            pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
            goal.poses.append(pose)
        self.sending = True
        future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.sending = False
        handle = future.result()
        if not handle.accepted:
            # O servidor existe antes de o lifecycle ativá-lo; o timer tenta
            # de novo a cada segundo.
            self.get_logger().info(
                'Nav2 ainda não ativo; tentando de novo.', once=True)
            return
        self.goal_handle = handle
        self.start_timer.cancel()
        self.destroy_timer(self.start_timer)
        self.start_timer = None
        self.start_time = self.now_s()
        self.last_truth_xy = self.truth[:2] if self.truth else None
        if self.truth is None:
            self.get_logger().warn(
                'Sem /ground_truth/odom: as colunas real_* ficam vazias.')
        self.get_logger().info(f'Missão {self.case} iniciada.')
        self.set_current(0)
        handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message):
        index = feedback_message.feedback.current_waypoint
        if index != self.current and 0 <= index < len(self.waypoints):
            # O follower só passa ao próximo depois da parada no anterior.
            self.close_waypoint(self.current)
            self.set_current(index)

    def result_callback(self, future):
        result = future.result().result
        self.close_waypoint(self.current)
        missed = set()
        for item in result.missed_waypoints:
            missed.add(int(getattr(item, 'index', item)))
        for i in range(len(self.waypoints)):
            if self.records[i] is None:
                self.records[i] = self.make_record(i)
            self.status[i] = MISSED if i in missed else REACHED
            self.records[i]['status'] = self.status[i]
        self.finished = True
        self.publish_markers()
        self.write_csv()

    # Registro --------------------------------------------------------------

    def set_current(self, index):
        self.current = index
        self.status[index] = CURRENT
        wp = self.waypoints[index]
        self.get_logger().info(
            f'Indo para o waypoint {index + 1}/{len(self.waypoints)} '
            f'"{wp["name"]}" ({wp["x"]:.2f}, {wp["y"]:.2f})')
        self.publish_markers()

    def close_waypoint(self, index):
        if index < 0 or self.records[index] is not None:
            return
        self.records[index] = self.make_record(index)
        self.status[index] = REACHED   # provisório: o resultado diz os perdidos
        self.publish_markers()

    def make_record(self, index):
        wp = self.waypoints[index]
        est = self.estimated_pose()
        real = self.truth
        row = {
            'indice': index + 1, 'nome': wp['name'], 'status': '',
            'alvo_x': wp['x'], 'alvo_y': wp['y'], 'alvo_yaw': wp['yaw'],
            't_saida_s': self.now_s() - self.start_time,
            'distancia_real_m': self.truth_distance if real else '',
        }
        if est:
            row.update(est_x=est[0], est_y=est[1], est_yaw=est[2],
                       erro_est_alvo_m=math.hypot(est[0] - wp['x'],
                                                  est[1] - wp['y']))
        if real:
            row.update(real_x=real[0], real_y=real[1], real_yaw=real[2],
                       erro_real_alvo_m=math.hypot(real[0] - wp['x'],
                                                   real[1] - wp['y']))
        if est and real:
            row.update(
                erro_localizacao_m=math.hypot(est[0] - real[0],
                                              est[1] - real[1]),
                erro_localizacao_yaw_rad=abs(wrap_angle(est[2] - real[2])))
        return row

    def record_trajectory(self):
        if self.start_time is None or self.finished:
            return
        est = self.estimated_pose()
        real = self.truth
        self.trajectory_rows.append({
            't_s': self.now_s() - self.start_time,
            'waypoint_atual': self.current + 1,
            'est_x': est[0] if est else '', 'est_y': est[1] if est else '',
            'est_yaw': est[2] if est else '',
            'real_x': real[0] if real else '', 'real_y': real[1] if real else '',
            'real_yaw': real[2] if real else '',
        })

    def write_csv(self):
        os.makedirs(self.output_dir, exist_ok=True)
        prefix = os.path.join(
            self.output_dir,
            f'{self.case}_{self.run_label}_{time.strftime("%Y%m%d-%H%M%S")}')
        for suffix, fields, rows in (
                ('waypoints', WAYPOINT_FIELDS, self.records),
                ('trajetoria', TRAJECTORY_FIELDS, self.trajectory_rows)):
            with open(f'{prefix}_{suffix}.csv', 'w', newline='') as out:
                writer = csv.DictWriter(out, fieldnames=fields)
                writer.writeheader()
                for row in rows:
                    writer.writerow({
                        k: (f'{v:.4f}' if isinstance(v, float) else v)
                        for k, v in row.items()})
        reached = self.status.count(REACHED)
        last = self.records[-1]
        final = last.get('erro_real_alvo_m')
        final_text = ('' if final is None
                      else f', erro real no último: {final:.2f} m')
        self.get_logger().info(
            f'Missão {self.case} terminada: {reached}/{len(self.waypoints)} '
            f'waypoints em {last["t_saida_s"]:.0f} s{final_text}. '
            f'CSV: {prefix}_*.csv')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # RViz ------------------------------------------------------------------

    def publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        line = Marker()
        line.header.frame_id = self.frame
        line.ns = 'rota'
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 1.0, 1.0, 0.6
        for i, wp in enumerate(self.waypoints):
            r, g, b = COLORS[self.status[i]]
            sphere = Marker()
            sphere.header.frame_id = self.frame
            sphere.ns = 'waypoints'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.pose.position.x = wp['x']
            sphere.pose.position.y = wp['y']
            sphere.pose.position.z = 0.1
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            sphere.color.r, sphere.color.g, sphere.color.b = r, g, b
            sphere.color.a = 0.9
            markers.markers.append(sphere)
            label = Marker()
            label.header.frame_id = self.frame
            label.ns = 'nomes'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = wp['x']
            label.pose.position.y = wp['y']
            label.pose.position.z = 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{i + 1} {wp["name"]}'
            markers.markers.append(label)
            line.points.append(sphere.pose.position)
        if len(line.points) > 1:
            markers.markers.append(line)
        self.marker_publisher.publish(markers)


def main():
    rclpy.init()
    node = WaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None and not node.finished:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
