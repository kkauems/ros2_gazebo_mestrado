#!/usr/bin/env python3
"""
Grava as fotos da câmera de aquisição de dados com a pose de cada uma.

Assina /data_camera/image_raw e, para cada imagem, guarda:
- a pose estimada da câmera: TF frame -> data_camera_optical_frame no
  carimbo da imagem (frame é map com SLAM/VSLAM, odom sem);
- a pose verdadeira: /ground_truth/odom do mesmo carimbo (Supervisor do
  Webots) composta com o TF fixo base_link -> câmera e trazida para o frame
  da partida, como no waypoint_mission. Ela só serve para medir o erro.

Só grava uma foto se a câmera andou pelo menos min_distance desde a última
gravada (pela pose estimada; sem ela, pela verdadeira), para não encher o
disco com o robô parado.

Saída, numa pasta por execução em output_dir (o launch passa fotos/ na raiz
do projeto; rodando o nó sozinho, o padrão é ~/.ros/agrobot_fotos):
    <mundo>_<modo>_<data>/
        000001.png, 000002.png, ...   RGB, sem perdas
        fotos.csv                     uma linha por foto
        camera_info.yaml              intrínsecos (iguais em todas as fotos)

Colunas de pose do fotos.csv (est_* estimada, real_* verdadeira):
    x, y, z          posição do centro óptico (m)
    roll, pitch, yaw orientação do data_camera_link (x saindo da lente), rad
    qx, qy, qz, qw   orientação do data_camera_optical_frame (z saindo da
                     lente, x para a direita, y para baixo), para projetar
                     pontos na imagem
"""

import csv
import math
import os
import struct
import time
import zlib

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

IMAGE_TOPIC = '/data_camera/image_raw'
INFO_TOPIC = '/data_camera/camera_info'
GROUND_TRUTH_TOPIC = '/ground_truth/odom'
CAMERA_FRAME = 'data_camera_optical_frame'

# Quanto esperar (tempo de simulação) pelo TF do carimbo da imagem. O
# map -> odom e o odom -> base_link chegam um pouco depois da imagem.
TF_WAIT = 1.0
# Imagens esperando o TF; com 1 imagem por segundo, 3 já é folga.
MAX_PENDING = 3
# Poses verdadeiras guardadas (uma por passo de 32 ms: ~3 s).
GROUND_TRUTH_KEEP = 100

POSE_FIELDS = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'qx', 'qy', 'qz', 'qw']
CSV_FIELDS = (
    ['foto', 'arquivo', 't_s', 'frame']
    + [f'est_{f}' for f in POSE_FIELDS]
    + [f'real_{f}' for f in POSE_FIELDS]
    + ['erro_xy_m', 'erro_z_m', 'erro_yaw_rad', 'distancia_anterior_m'])

# Rotação do data_camera_link para o frame óptico (rpy -pi/2 0 -pi/2 no
# URDF): colunas são os eixos do óptico escritos no link.
LINK_FROM_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(m):
    """Quaternion (x, y, z, w) de uma matriz de rotação 3x3, com w >= 0."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
             (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
             (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
             (m[1, 0] - m[0, 1]) / s)
    return tuple(-v for v in q) if q[3] < 0.0 else q


def matrix_to_rpy(m):
    """Roll, pitch, yaw (convenção do ROS: R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    roll = math.atan2(m[2, 1], m[2, 2])
    yaw = math.atan2(m[1, 0], m[0, 0])
    return roll, pitch, yaw


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_columns(position, rotation_optical):
    """Colunas de POSE_FIELDS para uma pose do frame óptico."""
    roll, pitch, yaw = matrix_to_rpy(
        rotation_optical @ LINK_FROM_OPTICAL.T)
    qx, qy, qz, qw = matrix_to_quaternion(rotation_optical)
    return dict(zip(POSE_FIELDS, (
        position[0], position[1], position[2], roll, pitch, yaw,
        qx, qy, qz, qw)))


def image_to_rgb(message):
    """Matriz altura x largura x 3 (RGB, uint8) de uma sensor_msgs/Image."""
    channels = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3}
    if message.encoding not in channels:
        raise ValueError(f'encoding {message.encoding} não suportado')
    n = channels[message.encoding]
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    rows = data.reshape(message.height, message.step)
    pixels = rows[:, :message.width * n].reshape(
        message.height, message.width, n)
    if message.encoding.startswith('bgr'):
        return pixels[:, :, 2::-1]
    return pixels[:, :, :3]


def write_png(path, rgb, level=6):
    """
    PNG RGB de 8 bits só com zlib e numpy (sem OpenCV nem PIL).

    Grava num arquivo temporário e renomeia, para nunca deixar uma foto pela
    metade se o launch for interrompido.
    """
    height, width, _ = rgb.shape
    raw = np.zeros((height, 1 + 3 * width), dtype=np.uint8)  # filtro 0
    raw[:, 1:] = np.ascontiguousarray(rgb).reshape(height, 3 * width)

    def chunk(kind, body):
        return (struct.pack('>I', len(body)) + kind + body
                + struct.pack('>I', zlib.crc32(kind + body) & 0xffffffff))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2,
                                        0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw.tobytes(), level))
           + chunk(b'IEND', b''))
    temporary = path + '.tmp'
    with open(temporary, 'wb') as out:
        out.write(png)
    os.replace(temporary, path)


def camera_info_yaml(info):
    """camera_info no formato do camera_calibration_parsers."""
    def matrix(name, rows, cols, values):
        data = ', '.join(f'{v:.6f}' for v in values)
        return (f'{name}:\n  rows: {rows}\n  cols: {cols}\n'
                f'  data: [{data}]\n')
    return (
        f'image_width: {info.width}\n'
        f'image_height: {info.height}\n'
        f'camera_name: data_camera\n'
        f'frame_id: {info.header.frame_id}\n'
        + matrix('camera_matrix', 3, 3, info.k)
        + f'distortion_model: {info.distortion_model}\n'
        + matrix('distortion_coefficients', 1, len(info.d), info.d)
        + matrix('rectification_matrix', 3, 3, info.r)
        + matrix('projection_matrix', 3, 4, info.p))


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class DataRecorder(Node):

    def __init__(self):
        super().__init__('data_recorder')
        self.declare_parameter('frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('run_label', '')
        self.declare_parameter('world', '')
        self.declare_parameter(
            'output_dir',
            os.path.join(os.path.expanduser('~'), '.ros', 'agrobot_fotos'))
        self.declare_parameter('min_distance', 0.3)
        self.declare_parameter('png_compression', 6)

        self.frame = self.get_parameter('frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        label = self.get_parameter('run_label').value or self.frame
        world = os.path.splitext(self.get_parameter('world').value)[0]
        name = '_'.join(
            p for p in (world, label, time.strftime('%Y%m%d-%H%M%S')) if p)
        self.folder = os.path.join(
            os.path.expanduser(self.get_parameter('output_dir').value), name)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.png_level = int(self.get_parameter('png_compression').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, reliable)
        self.create_subscription(
            CameraInfo, INFO_TOPIC, self.info_callback, reliable)
        self.create_subscription(
            Odometry, GROUND_TRUTH_TOPIC, self.ground_truth_callback, 50)

        self.camera_info = None
        self.ground_truth = {}         # carimbo (ns) -> (posição, rotação)
        self.truth_origin = None       # (posição, yaw) da partida no mundo
        self.base_to_camera = None     # TF fixo base_link -> óptico
        self.pending = []
        self.csv_file = None
        self.csv_writer = None
        self.saved = 0
        self.received = 0
        self.warned_no_image = False
        self.skipped = 0
        self.last_position = None
        self.create_timer(0.1, self.process_pending)
        self.get_logger().info(
            f'Gravando fotos em {self.folder} (uma a cada '
            f'{self.min_distance:.2f} m, poses em {self.frame}).')

    # Entradas ----------------------------------------------------------------

    def info_callback(self, message):
        self.camera_info = message

    def ground_truth_callback(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        position = np.array([p.x, p.y, p.z])
        rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        if self.truth_origin is None:
            # Frame da partida: o odom (e o map) nascem na pose inicial, só
            # com yaw (o EKF é 2D), como no waypoint_mission.
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            self.truth_origin = (position, yaw)
            if self.get_clock().now().nanoseconds > 5e9:
                self.get_logger().warn(
                    'A primeira pose verdadeira chegou depois de 5 s de '
                    'simulação: as colunas real_* são relativas a esse '
                    'momento, e não à partida do robô.')
        key = stamp_ns(message.header.stamp)
        self.ground_truth[key] = (position, rotation)
        if len(self.ground_truth) > GROUND_TRUTH_KEEP:
            del self.ground_truth[min(self.ground_truth)]

    def image_callback(self, message):
        self.received += 1
        self.pending.append(message)
        if len(self.pending) > MAX_PENDING:
            self.pending.pop(0)
            self.get_logger().warn(
                'Fila de fotos cheia: uma foto foi descartada sem gravar.')

    # Poses -------------------------------------------------------------------

    def estimated_pose(self, stamp):
        """(posição, rotação) do óptico no frame, ou None se ainda não deu."""
        transform = self.tf_buffer.lookup_transform(
            self.frame, CAMERA_FRAME, Time.from_msg(stamp))
        t = transform.transform.translation
        q = transform.transform.rotation
        return (np.array([t.x, t.y, t.z]),
                quaternion_to_matrix(q.x, q.y, q.z, q.w))

    def true_pose(self, stamp):
        """(posição, rotação) verdadeira do óptico no frame da partida."""
        truth = self.ground_truth.get(stamp_ns(stamp))
        if truth is None or self.truth_origin is None:
            return None
        if self.base_to_camera is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame, CAMERA_FRAME, Time())
            except TransformException:
                return None
            t = transform.transform.translation
            q = transform.transform.rotation
            self.base_to_camera = (np.array([t.x, t.y, t.z]),
                                   quaternion_to_matrix(q.x, q.y, q.z, q.w))
        position, rotation = truth
        offset, camera_rotation = self.base_to_camera
        world_position = position + rotation @ offset
        world_rotation = rotation @ camera_rotation
        origin, yaw0 = self.truth_origin
        back = rot_z(-yaw0)
        return back @ (world_position - origin), back @ world_rotation

    # Gravação ----------------------------------------------------------------

    def process_pending(self):
        now = self.get_clock().now()
        if (not self.received and not self.warned_no_image
                and now.nanoseconds > 15e9):
            self.warned_no_image = True
            self.get_logger().warn(
                f'Nenhuma imagem em {IMAGE_TOPIC} depois de 15 s de '
                'simulação: a câmera está ligada (data_camera:=true e '
                '/data_camera/enable)?')
        while self.pending:
            message = self.pending[0]
            stamp = Time.from_msg(message.header.stamp)
            try:
                estimated = self.estimated_pose(message.header.stamp)
            except TransformException as error:
                if now - stamp < Duration(seconds=TF_WAIT):
                    return     # o TF desse carimbo ainda pode chegar
                estimated = None
                self.get_logger().warn(
                    f'Sem TF {self.frame} -> {CAMERA_FRAME} no carimbo da '
                    f'foto ({error}); gravada sem a pose estimada.')
            self.pending.pop(0)
            self.handle(message, estimated)

    def handle(self, message, estimated):
        real = self.true_pose(message.header.stamp)
        reference = estimated or real
        distance = None
        if reference is not None and self.last_position is not None:
            distance = float(np.linalg.norm(
                reference[0][:2] - self.last_position[:2]))
            if distance < self.min_distance:
                self.skipped += 1
                return
        if self.camera_info is None:
            self.get_logger().warn(
                'Foto descartada: ainda sem /data_camera/camera_info.')
            return
        try:
            rgb = image_to_rgb(message)
        except ValueError as error:
            self.get_logger().error(f'Foto descartada: {error}')
            return
        self.open_output()
        self.saved += 1
        filename = f'{self.saved:06d}.png'
        write_png(os.path.join(self.folder, filename), rgb, self.png_level)
        if reference is not None:
            self.last_position = reference[0]

        row = {
            'foto': self.saved,
            'arquivo': filename,
            't_s': stamp_ns(message.header.stamp) * 1e-9,
            'frame': self.frame,
        }
        est = pose_columns(*estimated) if estimated else {}
        rea = pose_columns(*real) if real else {}
        for field in POSE_FIELDS:
            row[f'est_{field}'] = est.get(field, '')
            row[f'real_{field}'] = rea.get(field, '')
        if est and rea:
            row['erro_xy_m'] = math.hypot(
                est['x'] - rea['x'], est['y'] - rea['y'])
            row['erro_z_m'] = est['z'] - rea['z']
            row['erro_yaw_rad'] = wrap_angle(est['yaw'] - rea['yaw'])
        row['distancia_anterior_m'] = '' if distance is None else distance
        self.csv_writer.writerow({
            k: (f'{v:.4f}' if isinstance(v, float) else v)
            for k, v in row.items()})
        self.csv_file.flush()
        if self.saved == 1 or self.saved % 50 == 0:
            self.get_logger().info(
                f'{self.saved} fotos gravadas em {self.folder} '
                f'({self.skipped} puladas com o robô parado).')

    def open_output(self):
        if self.csv_file is not None:
            return
        os.makedirs(self.folder, exist_ok=True)
        with open(os.path.join(self.folder, 'camera_info.yaml'), 'w') as out:
            out.write(camera_info_yaml(self.camera_info))
        self.csv_file = open(
            os.path.join(self.folder, 'fotos.csv'), 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.get_logger().info(
                f'{self.saved} fotos gravadas em {self.folder}.')


def main():
    rclpy.init()
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
