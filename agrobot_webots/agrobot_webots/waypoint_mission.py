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
    map: olival_mapa1                # opcional: recusa rodar com outro mapa
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

Saídas, em output_dir (o launch passa dados/missions/ no projeto; rodando
o nó sozinho, o padrão é ~/.ros/agrobot_missions):
    <caso>_<modo>_<data>_waypoints.csv   uma linha por waypoint
    <caso>_<modo>_<data>_trajetoria.csv  pose estimada e verdadeira no tempo
A pose verdadeira vem de /ground_truth/odom (Supervisor do Webots), trazida
para o frame da partida. Ela só é usada para medir, nunca para navegar.

Com data_camera (o launch liga com data_camera:=true), a câmera de aquisição
fica desligada fora das missões: no início de cada missão o nó a liga
(/data_camera/enable) e manda o prefixo <caso>_<modo>_<data> dos CSVs para o
data_recorder (/data_recorder/session), que grava as fotos numa pasta com
esse nome; no fim, manda uma string vazia e desliga a câmera.

Interface web do operador (operator_bridge + página em web/): além do
RViz, a lista pode ser montada e comandada por
    ~/command  (std_msgs/String, JSON)  entrada: {"op": ...}
        set     troca a lista: {"op": "set", "name": "operador",
                "waypoints": [{"x": 1, "y": 2, "yaw": 0.5, "name": "a"}]}
        load    carrega um arquivo: {"op": "load", "case": "caso2_..."}
                (config/waypoints/ ou output_dir/, ou um caminho)
        start, cancel, clear, undo, check, save ({"name": opcional})
    ~/status   (std_msgs/String, JSON, latched)  saída: a lista com o
               estado e a checagem de cada waypoint, a missão em andamento,
               os arquivos de missão deste mundo e a última mensagem.
    ~/cancel   (std_srvs/Trigger)  cancela a missão em andamento.

Tópicos:
    /waypoints/markers   (visualization_msgs/MarkerArray)  saída, para o RViz
    /ground_truth/odom   (nav_msgs/Odometry)               entrada, opcional
    /clicked_point       (geometry_msgs/PointStamped)      entrada, do RViz
"""

import csv
import glob
import json
import math
import os
import time

from action_msgs.msg import GoalStatus
from agrobot_webots import waypoint_check
from ament_index_python.packages import (
    get_package_share_directory, PackageNotFoundError)
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

PENDING = 'pendente'
CURRENT = 'atual'
REACHED = 'atingido'
MISSED = 'perdido'
INVALID = 'invalido'   # o robô não cabe ou não gira ali (waypoint_check)
CANCELED = 'cancelado'  # a missão foi cancelada antes de chegar nele

COLORS = {
    PENDING: (1.0, 0.65, 0.0),
    CURRENT: (0.1, 0.5, 1.0),
    REACHED: (0.1, 0.8, 0.2),
    MISSED: (0.9, 0.1, 0.1),
    INVALID: (0.7, 0.1, 0.9),
    CANCELED: (0.5, 0.5, 0.5),
}

WAYPOINT_FIELDS = [
    'indice', 'nome', 'status', 'alvo_x', 'alvo_y', 'alvo_yaw',
    't_saida_s', 'est_x', 'est_y', 'est_yaw', 'real_x', 'real_y', 'real_yaw',
    'erro_est_alvo_m', 'erro_real_alvo_m', 'erro_localizacao_m',
    'erro_localizacao_yaw_rad', 'distancia_real_m',
]
CAMERA_ENABLE_SERVICE = '/data_camera/enable'
RECORDER_SESSION_TOPIC = '/data_recorder/session'

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
    return data.get('world'), data.get('map'), waypoints


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
        # Nome do mapa do RTAB-Map em uso (vazio sem VSLAM). Os waypoints
        # marcados num mapa só valem nele: o frame map de outro mapa é outro.
        self.declare_parameter('map', '')
        self.declare_parameter(
            'output_dir',
            os.path.join(os.path.expanduser('~'), '.ros', 'agrobot_missions'))
        self.declare_parameter('trajectory_rate', 5.0)
        # Liga a câmera de aquisição e a gravação das fotos só durante as
        # missões (data_camera:=true no launch).
        self.declare_parameter('data_camera', False)
        # Checagem dos waypoints pelas medidas do robô (waypoint_check.py):
        # recusa um ponto onde a pegada bate num obstáculo ou onde o robô não
        # consegue girar da direção em que chega até a de saída. A grade é o
        # /rtabmap/map com VSLAM ou o costmap global sem ele.
        self.declare_parameter('check_waypoints', True)
        self.declare_parameter('check_map_topic', '/global_costmap/costmap')
        # Valor a partir do qual a célula é obstáculo: 65 numa grade de
        # SLAM; 100 (letal) no costmap, para não contar a inflação.
        self.declare_parameter('check_occupied', 100)
        self.declare_parameter('robot_length', 1.2)
        self.declare_parameter('robot_width', 1.12)
        self.declare_parameter('check_margin', 0.05)
        # Quanto esperar a grade antes de começar uma missão de arquivo (s).
        self.declare_parameter('check_wait', 20.0)

        path = self.get_parameter('waypoints_file').value
        self.frame = self.get_parameter('frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.run_label = self.get_parameter('run_label').value or self.frame
        self.output_dir = self.get_parameter('output_dir').value
        self.world = self.get_parameter('world').value
        self.map = self.get_parameter('map').value
        if path:
            self.case = os.path.splitext(os.path.basename(path))[0]
            file_world, file_map, self.waypoints = load_waypoints(path)
        else:
            self.case = 'rviz'
            file_world, file_map, self.waypoints = None, None, []
        blocked = bool(file_world and self.world and file_world != self.world)
        if blocked:
            self.get_logger().error(
                f'{self.case} é para o mundo {file_world}, mas o mundo é '
                f'{self.world}. Missão cancelada.')
        if file_map and self.map and file_map != self.map:
            blocked = True
            self.get_logger().error(
                f'{self.case} foi marcado no mapa {file_map}, mas o mapa em '
                f'uso é {self.map} (vslam_db:={file_map}). Missão cancelada.')
        elif file_map and not self.map:
            self.get_logger().warn(
                f'{self.case} foi marcado no mapa {file_map}, mas o VSLAM '
                'está desligado: as coordenadas valem a partir da partida, '
                'sem a correção do mapa.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.check = bool(self.get_parameter('check_waypoints').value)
        self.grid = None
        self.grid_frame = None
        self.footprint = waypoint_check.Footprint(
            float(self.get_parameter('robot_length').value),
            float(self.get_parameter('robot_width').value),
            float(self.get_parameter('check_margin').value))
        self.check_occupied = int(self.get_parameter('check_occupied').value)
        self.check_wait = float(self.get_parameter('check_wait').value)
        self.wait_since = None
        if self.check:
            topic = self.get_parameter('check_map_topic').value
            # Volátil: aceita publicador volátil ou latched.
            self.create_subscription(
                OccupancyGrid, topic, self.grid_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            self.get_logger().info(
                f'Waypoints conferidos na grade {topic} (robô '
                f'{self.footprint.length:.2f} x {self.footprint.width:.2f} m '
                f'+ {self.footprint.margin:.2f} m de folga).')
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
        self.setup_operator()

        self.data_camera = bool(self.get_parameter('data_camera').value)
        if self.data_camera:
            self.camera_client = self.create_client(
                SetBool, CAMERA_ENABLE_SERVICE)
            self.session_publisher = self.create_publisher(
                String, RECORDER_SESSION_TOPIC, QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
            # Estado pedido para a câmera e o último confirmado pelo driver.
            # Os pedidos vão um de cada vez, pelo timer, para um "desliga"
            # atrasado nunca chegar depois do "liga" da missão. A câmera
            # nasce ligada no driver; fica desligada até a missão começar.
            self.camera_wanted = False
            self.camera_state = None
            self.camera_call = None
            self.create_timer(0.5, self.sync_camera)

        self.prefix = None
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

    # Checagem pelas medidas do robô --------------------------------------

    def grid_callback(self, message):
        info = message.info
        self.grid = waypoint_check.Grid(
            message.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            yaw_from_quaternion(info.origin.orientation),
            self.check_occupied)
        self.grid_frame = message.header.frame_id or self.frame

    def check_point(self, start, x, y, yaw_out):
        """
        Confere o ponto (x, y) do frame da missão: (status, texto, sugestão).

        start: de onde o robô vem (o waypoint anterior ou a pose dele).

        Sem grade (ou sem TF até ela) o ponto passa como desconhecido.
        """
        if self.grid is None:
            return waypoint_check.UNKNOWN, 'ainda sem mapa', None
        offset = (0.0, 0.0, 0.0)
        if self.grid_frame != self.frame:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.grid_frame, self.frame, Time())
            except TransformException:
                return waypoint_check.UNKNOWN, 'sem TF até o mapa', None
            offset = (t.transform.translation.x, t.transform.translation.y,
                      yaw_from_quaternion(t.transform.rotation))
        c, s = math.cos(offset[2]), math.sin(offset[2])
        gx = offset[0] + c * x - s * y
        gy = offset[1] + s * x + c * y
        gs = (offset[0] + c * start[0] - s * start[1],
              offset[1] + s * start[0] + c * start[1])
        gout = yaw_out + offset[2]
        status, angle, gin = waypoint_check.check_leg(
            self.grid, self.footprint, gs, gx, gy, gout)
        yaw_in = gin - offset[2]
        if status == waypoint_check.OK:
            return status, '', None
        if status == waypoint_check.UNKNOWN:
            return status, 'área ainda não mapeada', None
        if status == waypoint_check.UNREACHABLE:
            return (status, 'não há caminho por onde o robô passe até ele',
                    None)
        if status == waypoint_check.BLOCKED:
            text = (f'o robô não cabe ali chegando a '
                    f'{math.degrees(yaw_in):.0f}°')
        else:
            text = (f'o robô não consegue girar de '
                    f'{math.degrees(yaw_in):.0f}° para '
                    f'{math.degrees(yaw_out):.0f}° (bate a '
                    f'{math.degrees(wrap_angle(angle - offset[2])):.0f}°)')
        found = waypoint_check.suggest(
            self.grid, self.footprint, gx, gy, gin, gout)
        suggestion = None
        if found is not None:
            # De volta ao frame da missão.
            dx, dy = found[0] - offset[0], found[1] - offset[1]
            suggestion = (c * dx + s * dy, -s * dx + c * dy)
        return status, text, suggestion

    def leg_start(self, index, origin):
        """De onde o robô vem para o waypoint index."""
        if index > 0:
            prev = self.waypoints[index - 1]
            return prev['x'], prev['y']
        return origin

    def describe(self, index, text, suggestion):
        wp = self.waypoints[index]
        line = (f'Waypoint {index + 1} "{wp["name"]}" ({wp["x"]:.2f}, '
                f'{wp["y"]:.2f}): {text}.')
        if suggestion is not None:
            line += (f' Ponto livre mais perto: ({suggestion[0]:.2f}, '
                     f'{suggestion[1]:.2f}).')
        elif 'caminho' not in text:
            line += ' Nenhum ponto livre a até 1 m.'
        return line

    def validate_mission(self):
        """
        Confere todos os waypoints (já com yaw).

        Devolve True se a missão pode começar; marca os inválidos e explica
        no log.
        """
        est = self.estimated_pose()
        origin = est[:2] if est else (0.0, 0.0)
        bad, unknown = [], []
        for i, wp in enumerate(self.waypoints):
            status, text, suggestion = self.check_point(
                self.leg_start(i, origin), wp['x'], wp['y'], wp['yaw'])
            if status == waypoint_check.UNKNOWN:
                unknown.append(f'{i + 1} "{wp["name"]}"')
            elif status != waypoint_check.OK:
                bad.append(self.describe(i, text, suggestion))
                self.status[i] = INVALID
        if unknown:
            self.get_logger().warn(
                'Fora da área já mapeada, sem checagem: waypoints '
                + ', '.join(unknown) + '.')
        if bad:
            for line in bad:
                self.get_logger().error(line)
            self.get_logger().error(
                f'Missão {self.case} não iniciada: {len(bad)} waypoint(s) '
                'onde o robô não cabe, não gira ou não chega (em roxo no '
                'RViz). Mude os pontos (ou o yaw) e rode de novo; '
                'check_waypoints:=false desliga a checagem.')
            self.publish_markers()
            return False
        return True

    # Câmera de aquisição -------------------------------------------------

    def sync_camera(self):
        if self.camera_call is not None:
            if not self.camera_call.done():
                return
            call, self.camera_call = self.camera_call, None
            try:
                ok = bool(call.result().success)
            except Exception as error:  # noqa: B902 - só para avisar no log
                self.get_logger().warn(f'{CAMERA_ENABLE_SERVICE}: {error}')
                ok = False
            if ok:
                self.camera_state = self.camera_sent
                if not self.camera_sent and self.start_time is None:
                    self.get_logger().info(
                        'Câmera de aquisição desligada até o início da '
                        'missão.')
        if self.camera_state == self.camera_wanted:
            return
        if not self.camera_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = self.camera_wanted
        self.camera_sent = self.camera_wanted
        self.camera_call = self.camera_client.call_async(request)

    def set_camera(self, enabled):
        self.camera_wanted = enabled
        self.sync_camera()

    def start_recording(self):
        if not self.data_camera:
            return
        self.set_camera(True)
        self.session_publisher.publish(String(data=self.prefix))
        self.get_logger().info(
            f'Gravação das fotos ligada: dados/fotos/{self.prefix}/')

    def stop_recording(self):
        if not self.data_camera:
            return
        self.session_publisher.publish(String(data=''))
        self.set_camera(False)

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
        candidate = {'name': f'wp{n}', 'x': x, 'y': y, 'yaw_in': None}
        if self.check and not self.click_allowed(candidate):
            return
        self.waypoints.append(candidate)
        self.reset_progress()
        self.get_logger().info(
            f'Waypoint {n} acrescentado: ({x:.2f}, {y:.2f}) em {self.frame}')

    def click_allowed(self, candidate):
        """
        Confere um clique antes de acrescentá-lo.

        O robô tem de caber no ponto novo, chegando do anterior, e conseguir
        girar no anterior para vir até ele.
        """
        est = self.estimated_pose()
        origin = est[:2] if est else (0.0, 0.0)
        points = self.waypoints + [candidate]
        fill_yaw(points, origin)
        saved, self.waypoints = self.waypoints, points
        try:
            checks = [len(points) - 1]
            if len(points) > 1 and points[-2]['yaw_in'] is None:
                checks.insert(0, len(points) - 2)
            for i in checks:
                wp = points[i]
                status, text, suggestion = self.check_point(
                    self.leg_start(i, origin), wp['x'], wp['y'], wp['yaw'])
                if status in (waypoint_check.BLOCKED, waypoint_check.NO_TURN,
                              waypoint_check.UNREACHABLE):
                    line = self.describe(i, text, suggestion)
                    if i < len(points) - 1:
                        line += ' (para seguir até o ponto clicado)'
                    self.get_logger().warn(f'Clique recusado. {line}')
                    return False
                if status == waypoint_check.UNKNOWN and i == len(points) - 1:
                    self.get_logger().warn(
                        f'Ponto ({candidate["x"]:.2f}, {candidate["y"]:.2f})'
                        f' sem checagem: {text}.')
        finally:
            self.waypoints = saved
        return True

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
        response.success, response.message = self.save_list('rviz')
        return response

    def save_list(self, name):
        """Grava a lista em output_dir/<name>_<data>.yaml: (ok, caminho)."""
        if not self.waypoints:
            return False, 'lista vazia'
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(
            self.output_dir, f'{name}_{time.strftime("%Y%m%d-%H%M%S")}.yaml')
        lines = ['# Gravado pelo waypoint_mission.',
                 f'# Coordenadas no frame {self.frame}, a partir da partida.']
        if self.world:
            lines.append(f'world: {self.world}')
        if self.map:
            lines.append(f'map: {self.map}')
        lines.append('waypoints:')
        for wp in self.waypoints:
            yaw = '' if wp['yaw_in'] is None else f', yaw: {wp["yaw_in"]:.4f}'
            lines.append(f'  - {{name: {wp["name"]}, x: {wp["x"]:.3f}, '
                         f'y: {wp["y"]:.3f}{yaw}}}')
        with open(path, 'w') as out:
            out.write('\n'.join(lines) + '\n')
        self.get_logger().info(f'Waypoints gravados em {path}')
        self.cases = None   # a lista de arquivos mudou
        return True, path

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
        if self.check:
            # Espera a grade um pouco (o RTAB-Map e o costmap demoram a
            # publicar a primeira) e confere os waypoints antes de mandar.
            now = self.now_s()
            if self.wait_since is None:
                self.wait_since = now
            if self.grid is None and now - self.wait_since < self.check_wait:
                return
            self.wait_since = None
            if not self.validate_mission():
                self.start_timer.cancel()
                self.destroy_timer(self.start_timer)
                self.start_timer = None
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
        # Prefixo comum aos CSVs e à pasta das fotos, com a hora do início.
        self.prefix = (f'{self.case}_{self.run_label}_'
                       f'{time.strftime("%Y%m%d-%H%M%S")}')
        self.start_recording()
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
        wrapped = future.result()
        result = wrapped.result
        canceled = wrapped.status == GoalStatus.STATUS_CANCELED
        if not canceled:
            self.close_waypoint(self.current)
        missed = set()
        for item in result.missed_waypoints:
            missed.add(int(getattr(item, 'index', item)))
        for i in range(len(self.waypoints)):
            if self.records[i] is None:
                # Cancelada: os que faltavam (e o atual) não foram atingidos.
                self.records[i] = self.make_record(i)
                if canceled:
                    self.status[i] = CANCELED
            if self.status[i] != CANCELED:
                self.status[i] = MISSED if i in missed else REACHED
            self.records[i]['status'] = self.status[i]
        reached = self.status.count(REACHED)
        self.set_message(
            f'Missão {"cancelada" if canceled else "terminada"}: {reached}/'
            f'{len(self.waypoints)} waypoints atingidos.')
        self.finished = True
        self.stop_recording()
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
        prefix = os.path.join(self.output_dir, self.prefix)
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
        self.publish_status()

    # Interface web do operador -------------------------------------------

    def setup_operator(self):
        """Tópicos ~/command e ~/status e serviço ~/cancel (para a página)."""
        self.message = ''
        self.message_level = 'info'
        self.checks = []          # checagem de cada waypoint, para a página
        self.checked_key = None   # grade e lista da última checagem
        self.cases = None         # arquivos de missão deste mundo (cache)
        self.status_publisher = self.create_publisher(
            String, '~/status', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, '~/command', self.command_callback, 10)
        self.create_service(Trigger, '~/cancel', self.cancel_service)
        # Refaz a checagem quando a grade muda (o mapa cresce no mapeamento).
        self.create_timer(3.0, self.recheck)

    def set_message(self, text, level='info'):
        self.message = text
        self.message_level = level
        log = {'info': self.get_logger().info, 'warn': self.get_logger().warn,
               'error': self.get_logger().error}[level]
        log(text)
        self.publish_status()

    def command_callback(self, message):
        try:
            command = json.loads(message.data)
            op = command['op']
        except (ValueError, KeyError, TypeError) as error:
            self.set_message(f'Comando inválido: {error}', 'error')
            return
        if op == 'cancel':
            self.cancel_mission()
            return
        if op == 'check':
            self.run_checks()
            self.publish_status()
            return
        if op == 'save':
            ok, text = self.save_list(safe_name(command.get('name'), 'operador'))
            self.set_message(f'Lista gravada em {text}' if ok else text,
                             'info' if ok else 'warn')
            return
        if self.running():
            self.set_message(
                'Missão em andamento: cancele antes de mudar a lista.', 'warn')
            return
        if op == 'start':
            if self.start_mission():
                self.set_message(
                    f'Missão {self.case}: esperando o Nav2 e a checagem...')
            else:
                self.set_message('Lista vazia.', 'warn')
        elif op == 'set':
            try:
                points = parse_points(command.get('waypoints') or [])
            except (ValueError, KeyError, TypeError) as error:
                self.set_message(f'Lista inválida: {error}', 'error')
                return
            self.waypoints = points
            self.case = safe_name(command.get('name'), 'operador')
            self.after_edit()
        elif op == 'load':
            self.load_case(str(command.get('case', '')))
        elif op == 'clear':
            self.waypoints = []
            self.after_edit()
        elif op == 'undo':
            if self.waypoints:
                self.waypoints.pop()
            self.after_edit()
        else:
            self.set_message(f'Comando desconhecido: {op}', 'error')

    def after_edit(self):
        self.reset_progress()
        self.run_checks()
        self.message = f'{len(self.waypoints)} waypoints em {self.case}.'
        self.message_level = 'info'
        self.publish_status()

    def load_case(self, name):
        path = self.find_case(name)
        if path is None:
            self.set_message(f'Arquivo de missão não encontrado: {name}',
                             'error')
            return
        try:
            file_world, file_map, points = load_waypoints(path)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.set_message(f'Erro ao ler {path}: {error}', 'error')
            return
        if file_world and self.world and file_world != self.world:
            self.set_message(
                f'{name} é para o mundo {file_world}, e o mundo é '
                f'{self.world}.', 'error')
            return
        if file_map and self.map and file_map != self.map:
            self.set_message(
                f'{name} foi marcado no mapa {file_map}, e o mapa em uso é '
                f'{self.map}.', 'error')
            return
        self.waypoints = points
        self.case = os.path.splitext(os.path.basename(path))[0]
        self.after_edit()

    def case_dirs(self):
        dirs = [self.output_dir]
        try:
            dirs.insert(0, os.path.join(
                get_package_share_directory('agrobot_webots'),
                'config', 'waypoints'))
        except PackageNotFoundError:
            pass
        return dirs

    def find_case(self, name):
        if not name:
            return None
        if os.path.isfile(name):
            return name
        file_name = name if name.endswith('.yaml') else name + '.yaml'
        for folder in self.case_dirs():
            path = os.path.join(folder, os.path.basename(file_name))
            if os.path.isfile(path):
                return path
        return None

    def list_cases(self):
        """Arquivos de missão que valem neste mundo e neste mapa."""
        if self.cases is not None:
            return self.cases
        cases = []
        for folder in self.case_dirs():
            for path in sorted(glob.glob(os.path.join(folder, '*.yaml'))):
                try:
                    file_world, file_map, points = load_waypoints(path)
                except (OSError, ValueError, KeyError, TypeError, AttributeError,
                        yaml.YAMLError):
                    continue
                if file_world and self.world and file_world != self.world:
                    continue
                if file_map and self.map and file_map != self.map:
                    continue
                cases.append({
                    'name': os.path.splitext(os.path.basename(path))[0],
                    'waypoints': len(points),
                    'saved': folder == self.output_dir,
                })
        self.cases = cases
        return cases

    def cancel_service(self, request, response):
        response.success = self.cancel_mission()
        response.message = ('cancelando' if response.success
                            else 'nenhuma missão em andamento')
        return response

    def cancel_mission(self):
        if self.start_timer is not None:
            # Ainda esperando o Nav2: só para de tentar.
            self.start_timer.cancel()
            self.destroy_timer(self.start_timer)
            self.start_timer = None
            self.wait_since = None
            self.reset_progress()
            self.set_message('Início da missão cancelado.')
            return True
        if self.goal_handle is None or self.finished:
            self.set_message('Nenhuma missão em andamento.', 'warn')
            return False
        self.goal_handle.cancel_goal_async()
        self.set_message('Cancelando a missão...')
        return True

    def run_checks(self):
        """Confere cada waypoint (como no início da missão), para mostrar."""
        self.checks = []
        self.checked_key = self.check_key()
        if not self.waypoints:
            return
        est = self.estimated_pose()
        origin = est[:2] if est else (0.0, 0.0)
        fill_yaw(self.waypoints, origin)
        for i, wp in enumerate(self.waypoints):
            if not self.check:
                self.checks.append({'status': waypoint_check.OK, 'text': ''})
                continue
            status, text, suggestion = self.check_point(
                self.leg_start(i, origin), wp['x'], wp['y'], wp['yaw'])
            entry = {'status': status, 'text': text}
            if suggestion is not None:
                entry['suggestion'] = [round(suggestion[0], 3),
                                       round(suggestion[1], 3)]
            self.checks.append(entry)

    def check_key(self):
        return (id(self.grid), tuple(
            (wp['x'], wp['y'], wp['yaw_in']) for wp in self.waypoints))

    def recheck(self):
        """Refaz a checagem se a grade ou a lista (RViz) mudou."""
        if not self.running() and self.check_key() != self.checked_key:
            self.run_checks()
            self.publish_status()

    def publish_status(self):
        if not hasattr(self, 'status_publisher'):
            return
        waypoints = []
        for i, wp in enumerate(self.waypoints):
            entry = {
                'name': wp['name'], 'x': round(wp['x'], 3),
                'y': round(wp['y'], 3),
                'yaw': (None if wp['yaw_in'] is None
                        else round(wp['yaw_in'], 4)),
                'heading': (round(wp['yaw'], 4) if 'yaw' in wp else None),
                'status': (self.status[i] if i < len(self.status)
                           else PENDING),
            }
            if i < len(self.checks):
                entry['check'] = self.checks[i]
            waypoints.append(entry)
        message, level = self.message, self.message_level
        if (self.goal_handle is not None and not self.finished
                and 0 <= self.current < len(self.waypoints)
                and not message.startswith('Cancelando')):
            wp = self.waypoints[self.current]
            message = (f'Missão {self.case}: indo para o waypoint '
                       f'{self.current + 1}/{len(self.waypoints)} '
                       f'"{wp["name"]}".')
            level = 'info'
        if (not self.running() and INVALID in self.status
                and self.start_time is None):
            message = ('Missão não iniciada: há waypoints onde o robô não '
                       'cabe, não gira ou não chega (em roxo).')
            level = 'error'
        status = {
            'case': self.case, 'frame': self.frame, 'world': self.world,
            'map': self.map, 'running': self.running(),
            'starting': self.start_timer is not None,
            'finished': self.finished, 'current': self.current,
            'prefix': self.prefix, 'check': self.check,
            'grid': self.grid is not None,
            'data_camera': self.data_camera,
            'message': message, 'level': level,
            'waypoints': waypoints, 'cases': self.list_cases(),
        }
        self.status_publisher.publish(String(data=json.dumps(status)))


def safe_name(name, default):
    """Nome para arquivos: letras, números, '-' e '_' (sem espaços)."""
    text = ''.join(c if c.isalnum() or c in '-_' else '-'
                   for c in str(name or '').strip())
    return text.strip('-') or default


def parse_points(entries):
    """Lista da página ([{x, y, yaw?, name?}]) no formato interno."""
    points = []
    for i, entry in enumerate(entries):
        x, y = float(entry['x']), float(entry['y'])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'waypoint {i + 1} sem coordenadas')
        yaw = entry.get('yaw')
        points.append({
            'name': safe_name(entry.get('name'), f'wp{i + 1}'),
            'x': x, 'y': y,
            'yaw_in': None if yaw is None else float(yaw),
        })
    return points


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
            if node.data_camera and rclpy.ok():
                node.stop_recording()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
