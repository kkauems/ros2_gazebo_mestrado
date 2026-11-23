#!/usr/bin/env python3
"""
Rumo pelas fileiras: mede o yaw do robô pelo ângulo das fileiras no LiDAR 3D.

O yaw do EKF vem só do giroscópio, que deriva (bias de ~0,01 grau/s e erro de
escala de 1%). Numa passada que não volta por onde já passou, o RTAB-Map não
tem com que fechar laço e não corrige essa deriva: no caso 4 do olival, 1,8
grau na ida viravam 1,7 m de erro lateral na cabeceira leste. Num pomar ou
numa lavoura as fileiras são retas e paralelas, e os troncos, as copas e as
plantas formam linhas que o LiDAR 3D vê a dezenas de metros. O ângulo delas
no frame do robô dá o yaw do robô em relação às fileiras, sem deriva.

Como mede (heading_from_points):
- os pontos da nuvem a 1 até range_max m do robô e de min_height a
  max_height acima do chão local (o menor z numa grade de 1 m: o chão do
  olival é inclinado e ondulado) são os obstáculos;
- para cada ângulo candidato perto do previsto, projeta os pontos na normal
  da direção e conta quantos caem em cada faixa de bin_width m. Quando a
  direção é a das fileiras, os pontos de cada fileira caem nas mesmas
  faixas e a soma dos quadrados das contagens é máxima;
- no olival a grade de 7 x 7 m tem linhas nas duas direções; a busca fica
  a até gate graus do yaw previsto pelo EKF, então vale qualquer múltiplo
  de 90 graus.

Como entra no EKF: publica /row_heading/pose (PoseWithCovarianceStamped no
frame odom, só o yaw vale) e o EKF a funde como medida absoluta de yaw no
lugar do yaw do IMU (o IMU continua dando a velocidade de giro). Hipótese:
fileiras retas e paralelas. A direção delas no odom (azimute) é medida nas
primeiras nuvens, com o robô parado na partida: não precisa estar alinhado.

Só publica com o robô quase sem girar (a nuvem de 0,1 s gira junto), quando
o pico é nítido (quality) e não fica na borda da busca, e quando concorda
com o EKF a até max_jump graus (a deriva do EKF entre duas medidas é bem
menor que isso).

Medido no banco de um caso 4 do olival (pose verdadeira como referência,
trechos retos): 0,5 grau rms por nuvem, sem viés (0,06 grau), contra 1,2
grau rms do EKF no mesmo trecho.

Tópicos:
    /lidar3d/points    (sensor_msgs/PointCloud2)  entrada
    /odom              (nav_msgs/Odometry)        entrada, yaw previsto
    /row_heading/pose  (geometry_msgs/PoseWithCovarianceStamped) saída
"""

from collections import deque
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros


QUARTER = math.pi / 2


def wrap(angle):
    """Ângulo em [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def obstacle_points(points, range_max=40.0, range_min=1.0, min_height=0.3,
                    max_height=4.0, cell=1.0):
    """
    Pontos (N, 2) dos obstáculos, de uma nuvem (N, 3) no base_link.

    A altura é medida a partir do chão local: o menor z de cada célula de
    cell m. Pontos mais baixos que min_height acima dele são o chão.
    """
    r = np.hypot(points[:, 0], points[:, 1])
    points = points[(r < range_max) & (r > range_min)]
    if len(points) == 0:
        return np.zeros((0, 2))
    ij = np.floor(points[:, :2] / cell).astype(np.int64)
    key = ij[:, 0] * 100003 + ij[:, 1]
    unique, inverse = np.unique(key, return_inverse=True)
    ground = np.full(len(unique), np.inf)
    np.minimum.at(ground, inverse, points[:, 2])
    height = points[:, 2] - ground[inverse]
    keep = (height > min_height) & (height < max_height)
    return points[keep, :2]


def heading_from_points(points, row_angle, gate=math.radians(8.0),
                        step=math.radians(0.1), bin_width=0.2,
                        range_max=40.0):
    """
    Direção das fileiras no frame do robô, perto de row_angle (rad).

    Devolve (ângulo, qualidade, na_borda): qualidade é o pico da nitidez
    sobre a mediana dos ângulos testados (1 = nenhuma direção se destaca).
    """
    angles = np.arange(row_angle - gate, row_angle + gate + 1e-9, step)
    bins = int(2 * range_max / bin_width) + 2
    scores = np.empty(len(angles))
    for i, angle in enumerate(angles):
        d = -points[:, 0] * math.sin(angle) + points[:, 1] * math.cos(angle)
        index = np.clip(((d + range_max) / bin_width).astype(np.int64),
                        0, bins - 1)
        counts = np.bincount(index, minlength=bins).astype(np.float64)
        scores[i] = (counts * counts).sum()
    best = int(scores.argmax())
    median = float(np.median(scores))
    quality = float(scores[best] / median) if median > 0 else 0.0
    return float(angles[best]), quality, best in (0, len(angles) - 1)


def row_yaw(robot_yaw, azimuth):
    """
    Direção das fileiras no frame do robô (o múltiplo de 90 graus mais perto).

    azimuth: direção das fileiras no odom. Devolve (ângulo no robô, k).
    """
    k = round((robot_yaw - azimuth) / QUARTER)
    return wrap(azimuth + k * QUARTER - robot_yaw), k


def yaw_from_rows(angle, azimuth, k):
    """Yaw do robô no odom, dada a direção das fileiras no frame dele."""
    return wrap(azimuth + k * QUARTER - angle)


class RowHeading(Node):
    """Mede o yaw pelas fileiras e publica para o EKF."""

    def __init__(self):
        super().__init__('row_heading')
        self.declare_parameter('cloud_topic', '/lidar3d/points')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('period', 0.4)          # s entre medidas
        self.declare_parameter('range_max', 40.0)
        self.declare_parameter('min_height', 0.3)
        self.declare_parameter('max_height', 4.0)
        self.declare_parameter('bin_width', 0.2)
        self.declare_parameter('gate_deg', 8.0)
        self.declare_parameter('quality', 1.2)
        self.declare_parameter('max_jump_deg', 3.0)
        self.declare_parameter('max_yaw_rate', 0.05)   # rad/s
        self.declare_parameter('min_points', 200)
        self.declare_parameter('calibration_samples', 5)
        # 1 grau de desvio por medida: a medida erra ~0,5 grau rms e o EKF
        # soma várias por segundo.
        self.declare_parameter('yaw_std_deg', 1.0)
        p = self.get_parameter
        self.base_frame = p('base_frame').value
        self.odom_frame = p('odom_frame').value
        self.period = p('period').value
        self.range_max = p('range_max').value
        self.min_height = p('min_height').value
        self.max_height = p('max_height').value
        self.bin_width = p('bin_width').value
        self.gate = math.radians(p('gate_deg').value)
        self.min_quality = p('quality').value
        self.max_jump = math.radians(p('max_jump_deg').value)
        self.max_yaw_rate = p('max_yaw_rate').value
        self.min_points = p('min_points').value
        self.calibration_samples = p('calibration_samples').value
        self.yaw_variance = math.radians(p('yaw_std_deg').value) ** 2

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cloud_to_base = None
        self.odom = None            # (yaw, yaw_rate) da última odometria
        self.history = deque(maxlen=300)   # (t, yaw) da odometria, ~10 s
        self.azimuth = None
        self.calibration = []
        self.last_time = None
        self.sent = self.rejected = 0
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/row_heading/pose', 10)
        self.create_subscription(Odometry, p('odom_topic').value,
                                 self.odom_callback, 20)
        self.create_subscription(PointCloud2, p('cloud_topic').value,
                                 self.cloud_callback, qos_profile_sensor_data)
        self.create_timer(60.0, self.log_stats)

    def odom_callback(self, message):
        self.odom = (yaw_of(message.pose.pose.orientation),
                     message.twist.twist.angular.z)
        self.history.append((
            Time.from_msg(message.header.stamp).nanoseconds * 1e-9,
            self.odom[0]))

    def log_stats(self):
        if self.sent or self.rejected:
            self.get_logger().info(
                f'Rumo pelas fileiras: {self.sent} medidas enviadas ao EKF, '
                f'{self.rejected} recusadas no último minuto.')
        self.sent = self.rejected = 0

    def cloud_points(self, cloud):
        if self.cloud_to_base is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, cloud.header.frame_id, Time())
            except tf2_ros.TransformException:
                return None
            q, v = t.transform.rotation, t.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w),
                 1 - 2 * (x * x + y * y)]])
            self.cloud_to_base = (rotation, np.array([v.x, v.y, v.z]))
        if cloud.point_step != 12:
            self.get_logger().error('A nuvem precisa ser x, y, z float32.',
                                    throttle_duration_sec=10.0)
            return None
        points = np.frombuffer(bytes(cloud.data), dtype=np.float32).reshape(
            -1, 3).astype(np.float64)
        rotation, translation = self.cloud_to_base
        return points @ rotation.T + translation

    def odom_yaw_at(self, t):
        """Yaw do EKF no instante t (a odometria mais perto, até 0,2 s)."""
        if not self.history:
            return None
        best = min(self.history, key=lambda item: abs(item[0] - t))
        if abs(best[0] - t) > 0.2:
            return None
        return best[1]

    def cloud_callback(self, cloud):
        now = Time.from_msg(cloud.header.stamp).nanoseconds * 1e-9
        if self.last_time is not None and now - self.last_time < self.period:
            return
        if self.odom is None or abs(self.odom[1]) > self.max_yaw_rate:
            return
        self.last_time = now
        yaw_then = self.odom_yaw_at(now)
        if yaw_then is None:
            self.get_logger().warn('Sem odometria no instante da nuvem.',
                                   throttle_duration_sec=30.0)
            return
        points = self.cloud_points(cloud)
        if points is None:
            self.get_logger().warn('Sem o TF da nuvem para o base_link.',
                                   throttle_duration_sec=30.0)
            return
        obstacles = obstacle_points(points, self.range_max, 1.0,
                                    self.min_height, self.max_height)
        if len(obstacles) < self.min_points:
            self.rejected += 1
            return
        if self.azimuth is None:
            self.calibrate(obstacles, yaw_then)
            return
        predicted, k = row_yaw(yaw_then, self.azimuth)
        angle, quality, edge = heading_from_points(
            obstacles, predicted, self.gate, bin_width=self.bin_width,
            range_max=self.range_max)
        measured = yaw_from_rows(angle, self.azimuth, k)
        if (edge or quality < self.min_quality
                or abs(wrap(measured - yaw_then)) > self.max_jump):
            self.rejected += 1
            return
        # Leva a medida para o instante da última odometria: o EKF já
        # passou do carimbo da nuvem enquanto ela era processada.
        yaw_now = self.odom[0]
        self.publish(wrap(measured + wrap(yaw_now - yaw_then)))
        self.sent += 1

    def calibrate(self, obstacles, yaw_then):
        # Direção das fileiras no odom: busca em todos os ângulos (de -45 a
        # 45 graus em volta da frente do robô) e tira a média circular.
        angle, quality, _ = heading_from_points(
            obstacles, 0.0, math.radians(45.0), bin_width=self.bin_width,
            range_max=self.range_max)
        if quality < self.min_quality:
            self.rejected += 1
            return
        self.calibration.append(wrap(yaw_then + angle))
        if len(self.calibration) < self.calibration_samples:
            return
        # Média em 4x o ângulo: as direções valem módulo 90 graus.
        c = np.mean([math.cos(4 * a) for a in self.calibration])
        s = np.mean([math.sin(4 * a) for a in self.calibration])
        self.azimuth = math.atan2(s, c) / 4
        self.get_logger().info(
            f'Fileiras a {math.degrees(self.azimuth):.2f} graus no odom '
            f'({len(self.calibration)} nuvens); o rumo pelas fileiras '
            'passa a corrigir o yaw do EKF.')

    def publish(self, yaw):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.pose.pose.orientation.z = math.sin(yaw / 2)
        message.pose.pose.orientation.w = math.cos(yaw / 2)
        covariance = [0.0] * 36
        for i in range(5):
            covariance[i * 7] = 1e6     # x, y, z, roll, pitch: não usados
        covariance[35] = self.yaw_variance
        message.pose.covariance = covariance
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = RowHeading()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
