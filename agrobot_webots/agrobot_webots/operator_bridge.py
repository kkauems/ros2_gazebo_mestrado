#!/usr/bin/env python3
"""
Prepara os dados do robô para a interface web do operador (web/).

A página fala com o ROS pelo rosbridge (WebSocket). Mandar a grade do mapa,
as nuvens e as imagens cruas por ele pesaria demais (JSON), então este nó
resume tudo em mensagens pequenas, só enquanto a página as assina:

    /operator/state            (std_msgs/String, JSON, state_rate Hz)
        pose do robô no frame global (TF), velocidade, quem comanda
        (cmd_vel_mux), patinagem, caminho do Nav2 (/plan) e o scan do
        LiDAR já no frame global, para desenhar no mapa
    /operator/map              (std_msgs/String, JSON, até map_rate Hz)
        a grade (/rtabmap/map com VSLAM, costmap global sem) como PNG em
        base64, com resolução, origem e tamanho
    /operator/vslam_camera/compressed   (sensor_msgs/CompressedImage, JPEG)
    /operator/vslam_depth/compressed    (idem, profundidade em cores)
    /operator/data_camera/compressed    (idem, câmera de aquisição reduzida)

O comando vai direto da página para os nós que já existem: /cmd_vel_teleop
(cmd_vel_mux), /waypoint_mission/command e /data_camera/enable.
"""

import base64
import json
import math
import time

from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import cv2
except ImportError:   # sem OpenCV: sem mapa e sem imagens na página
    cv2 = None

# Cores da grade no PNG (tons de cinza): livre, desconhecido, ocupado.
FREE = 245
UNKNOWN = 160
OCCUPIED = 30


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def grid_to_gray(data, width, height, occupied=65):
    """
    Grade de ocupação (linhas de y crescente) em imagem cinza.

    A primeira linha da imagem é o y máximo, para a página desenhar com o
    norte para cima. No costmap (0 a 100) o custo vira tons intermediários.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(int(height), int(width))
    image = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    known = grid >= 0
    # Livre (0) claro, ocupado (>= occupied) escuro, o meio em degradê.
    cost = np.clip(grid.astype(np.float32) / float(occupied), 0.0, 1.0)
    shade = FREE - cost * (FREE - OCCUPIED - 40)
    image[known] = shade[known].astype(np.uint8)
    image[grid >= occupied] = OCCUPIED
    return np.flipud(image)


def scan_points(ranges, angle_min, angle_increment, range_max, pose,
                step=3):
    """Pontos do scan no frame global, (x, y) achatados, a cada step raios."""
    x0, y0, yaw0 = pose
    points = []
    for i in range(0, len(ranges), step):
        r = ranges[i]
        if not math.isfinite(r) or r <= 0.0 or r >= range_max:
            continue
        a = yaw0 + angle_min + i * angle_increment
        points.append(round(x0 + r * math.cos(a), 2))
        points.append(round(y0 + r * math.sin(a), 2))
    return points


def decimate(points, limit):
    """No máximo limit pontos da lista, sempre com o último."""
    if len(points) <= limit:
        return points
    step = len(points) / float(limit - 1)
    out = [points[int(i * step)] for i in range(limit - 1)]
    out.append(points[-1])
    return out


def image_to_bgr(message):
    """sensor_msgs/Image (bgra8, bgr8, rgb8, rgba8) em matriz BGR."""
    channels = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3}
    n = channels.get(message.encoding)
    if n is None:
        raise ValueError(f'encoding {message.encoding} não suportado')
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    image = raw.reshape(message.height, message.step)[
        :, :message.width * n].reshape(message.height, message.width, n)
    if message.encoding.startswith('rgb'):
        image = image[:, :, [2, 1, 0]]
    return np.ascontiguousarray(image[:, :, :3])


def depth_to_bgr(message, near=0.5, far=6.0):
    """Profundidade 32FC1 (m) em cores: perto quente, longe frio."""
    raw = np.frombuffer(bytes(message.data), dtype=np.float32)
    depth = raw.reshape(message.height, message.step // 4)[:, :message.width]
    valid = np.isfinite(depth) & (depth > 0.0)
    scaled = np.zeros(depth.shape, dtype=np.uint8)
    clipped = np.clip(depth[valid], near, far)
    scaled[valid] = (255.0 * (far - clipped) / (far - near)).astype(np.uint8)
    colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored


class OperatorBridge(Node):

    def __init__(self):
        super().__init__('operator_bridge')
        self.declare_parameter('frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_topic', '/global_costmap/costmap')
        self.declare_parameter('map_occupied', 65)
        self.declare_parameter('state_rate', 5.0)
        self.declare_parameter('map_rate', 0.5)
        self.declare_parameter('jpeg_quality', 70)
        # Período mínimo (s) e largura máxima (px) de cada prévia.
        self.declare_parameter('vslam_camera_period', 0.2)
        self.declare_parameter('vslam_depth_period', 0.5)
        self.declare_parameter('data_camera_period', 1.0)
        self.declare_parameter('data_camera_width', 640)

        self.frame = self.get_parameter('frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_occupied = int(self.get_parameter('map_occupied').value)
        self.map_period = 1.0 / float(self.get_parameter('map_rate').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_publisher = self.create_publisher(
            String, '/operator/state', 10)
        self.map_publisher = self.create_publisher(
            String, '/operator/map', latched)

        self.twist = None
        self.source = 'none'
        self.slip = False
        self.plan = []
        self.plan_time = 0.0
        self.scan = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            String, '/cmd_vel_mux/source', self.source_callback, latched)
        self.create_subscription(
            Bool, '/wheel/slip', self.slip_callback, latched)
        # Camada de segurança: qual zona está agindo, no nó da parada
        # (/collision_monitor_state) e no do limite de velocidade; a parada
        # vale mais.
        self.safety_states = {'stop': '', 'limit': ''}
        self.safety = ''
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            lambda m: self.safety_callback('stop', m), 10)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_limit_state',
            lambda m: self.safety_callback('limit', m), 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.map_message = None
        self.map_sent = 0.0
        self.map_info = None
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.map_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        # Prévias das câmeras: (tópico de entrada, de saída, conversor).
        self.previews = {}
        if cv2 is None:
            self.get_logger().warn(
                'OpenCV (python3-opencv) não instalado: a página fica sem o '
                'mapa e sem as câmeras.')
        else:
            width = int(self.get_parameter('data_camera_width').value)
            for key, topic, convert in (
                    ('vslam_camera', '/vslam_camera/color/image_raw',
                     image_to_bgr),
                    ('vslam_depth',
                     '/vslam_camera/aligned_depth_to_color/image_raw',
                     depth_to_bgr),
                    ('data_camera', '/data_camera/image_raw',
                     lambda m, w=width: shrink(image_to_bgr(m), w))):
                self.add_preview(key, topic, convert)

        # Relógio de parede: com a simulação lenta, um timer no tempo
        # simulado deixaria a página travada (a 1 Hz em vez de 5).
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            1.0 / float(self.get_parameter('state_rate').value),
            self.publish_state, clock=wall)
        self.create_timer(0.5, self.publish_map, clock=wall)
        self.create_timer(1.0, self.sync_previews, clock=wall)
        self.get_logger().info(
            f'Ponte da interface web: frame {self.frame}, mapa '
            f'{self.get_parameter("map_topic").value}.')

    # Entradas ----------------------------------------------------------------

    def odom_callback(self, message):
        self.twist = message.twist.twist

    def source_callback(self, message):
        self.source = message.data

    def slip_callback(self, message):
        self.slip = bool(message.data)

    def safety_callback(self, monitor, message):
        # A zona de lentidão limita a velocidade (ação limit): para a página,
        # é "lentidão".
        actions = {CollisionMonitorState.STOP: 'parada',
                   CollisionMonitorState.SLOWDOWN: 'lentidao',
                   CollisionMonitorState.APPROACH: 'aproximacao',
                   CollisionMonitorState.LIMIT: 'lentidao'}
        self.safety_states[monitor] = actions.get(message.action_type, '')
        self.safety = (self.safety_states['stop']
                       or self.safety_states['limit'])

    def plan_callback(self, message):
        points = [(round(p.pose.position.x, 2), round(p.pose.position.y, 2))
                  for p in message.poses]
        if message.header.frame_id and message.header.frame_id != self.frame:
            points = self.transform_points(points, message.header.frame_id)
        self.plan = [c for p in decimate(points, 150) for c in p]
        self.plan_time = time.monotonic()

    def scan_callback(self, message):
        self.scan = message

    def map_callback(self, message):
        self.map_message = message

    # Saídas ------------------------------------------------------------------

    def lookup(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(t.transform.rotation))

    def transform_points(self, points, source):
        pose = self.lookup(self.frame, source)
        if pose is None:
            return []
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return [(round(pose[0] + c * x - s * y, 2),
                 round(pose[1] + s * x + c * y, 2)) for x, y in points]

    def publish_state(self):
        if self.state_publisher.get_subscription_count() == 0:
            return
        pose = self.lookup(self.frame, self.robot_frame)
        state = {
            'frame': self.frame,
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'pose': None if pose is None else [round(v, 3) for v in pose],
            'v': None if self.twist is None else round(self.twist.linear.x, 3),
            'w': None if self.twist is None else round(self.twist.angular.z, 3),
            'source': self.source,
            'slip': self.slip,
            # Ação do collision_monitor: '', parada, lentidao, aproximacao.
            'safety': self.safety,
            # O /plan só sai quando o Nav2 replaneja; velho, some da tela.
            'plan': (self.plan if time.monotonic() - self.plan_time < 5.0
                     else []),
            'scan': [],
            'map': self.map_info,
        }
        if self.scan is not None:
            scan_pose = self.lookup(self.frame, self.scan.header.frame_id)
            if scan_pose is not None:
                state['scan'] = scan_points(
                    self.scan.ranges, self.scan.angle_min,
                    self.scan.angle_increment, self.scan.range_max,
                    scan_pose)
        self.state_publisher.publish(String(data=json.dumps(state)))

    def publish_map(self):
        message = self.map_message
        if message is None or cv2 is None:
            return
        now = time.monotonic()
        if now - self.map_sent < self.map_period:
            return
        self.map_message = None
        self.map_sent = now
        info = message.info
        gray = grid_to_gray(message.data, info.width, info.height,
                            self.map_occupied)
        ok, png = cv2.imencode('.png', gray)
        if not ok:
            return
        self.map_info = {
            'frame': message.header.frame_id or self.frame,
            'resolution': info.resolution,
            'width': info.width, 'height': info.height,
            'origin': [info.origin.position.x, info.origin.position.y,
                       yaw_from_quaternion(info.origin.orientation)],
        }
        if self.map_info['frame'] != self.frame:
            # Costmap em outro frame: leva a origem para o frame da página.
            pose = self.lookup(self.frame, self.map_info['frame'])
            if pose is None:
                return
            x, y, yaw = self.map_info['origin']
            c, s = math.cos(pose[2]), math.sin(pose[2])
            self.map_info['origin'] = [pose[0] + c * x - s * y,
                                       pose[1] + s * x + c * y,
                                       yaw + pose[2]]
            self.map_info['frame'] = self.frame
        payload = dict(self.map_info)
        payload['png'] = base64.b64encode(png.tobytes()).decode('ascii')
        self.map_publisher.publish(String(data=json.dumps(payload)))

    def add_preview(self, key, topic, convert):
        out = self.create_publisher(
            CompressedImage, f'/operator/{key}/compressed', 1)
        period = float(self.get_parameter(f'{key}_period').value)
        self.previews[key] = {'last': 0.0, 'out': out, 'topic': topic,
                              'subscription': None}

        def callback(message):
            now = time.monotonic()
            preview = self.previews[key]
            if now - preview['last'] < period:
                return
            preview['last'] = now
            try:
                image = convert(message)
            except ValueError as error:
                self.get_logger().warn(f'{topic}: {error}', once=True)
                return
            ok, jpeg = cv2.imencode(
                '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                return
            compressed = CompressedImage()
            compressed.header = message.header
            compressed.format = 'jpeg'
            compressed.data = jpeg.tobytes()
            out.publish(compressed)

        self.previews[key]['callback'] = callback

    def sync_previews(self):
        """
        Assina cada câmera só enquanto a página assina a prévia.

        A câmera de aquisição manda 4,9 MB por imagem; recebê-la à toa pesa.
        Confiável (e não best effort): imagens grandes em best effort se
        perdem em pedaços e nunca chegam.
        """
        for key, preview in self.previews.items():
            wanted = preview['out'].get_subscription_count() > 0
            if wanted and preview['subscription'] is None:
                preview['subscription'] = self.create_subscription(
                    Image, preview['topic'], preview['callback'],
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            elif not wanted and preview['subscription'] is not None:
                self.destroy_subscription(preview['subscription'])
                preview['subscription'] = None


def shrink(image, width):
    """Reduz a imagem para no máximo width px de largura."""
    if image.shape[1] <= width:
        return image
    height = int(round(image.shape[0] * width / image.shape[1]))
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
