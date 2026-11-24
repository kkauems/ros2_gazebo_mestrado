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

Posição lateral pelas fileiras (lateral:=true, lateral_correction:=rows):
as fileiras também dizem onde o robô está entre elas. Na mesma nuvem, com a
direção já medida, os pontos projetados na normal das fileiras se juntam a
cada row_spacing m; a fase desse padrão (o pico do histograma módulo
row_spacing) é a posição das fileiras em relação ao robô. Na partida o nó
anota a fase das fileiras no odom; depois, a diferença entre a fase medida e
a anotada é o erro do EKF na direção perpendicular às fileiras (o que o robô
escorregou encosta abaixo e as rodas não viram). Publica a posição corrigida
em /row_heading/position, com variância pequena só nessa direção (ao longo
da fileira, 1e6: as fileiras não dizem nada). Vale enquanto o erro for menor
que meia entrelinha; aceita só correções até max_offset m. No olival a
grade de 7 x 7 m também dá a posição ao longo das fileiras nas cabeceiras
(andando para norte ou sul, as "fileiras" são as colunas de árvores, a cada
tree_spacing m).

Tópicos:
    /lidar3d/points        (sensor_msgs/PointCloud2)  entrada
    /odom                  (nav_msgs/Odometry)        entrada, pose prevista
    /row_heading/pose      (geometry_msgs/PoseWithCovarianceStamped) saída
    /row_heading/position  (geometry_msgs/PoseWithCovarianceStamped) saída,
                           só com lateral:=true
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


def row_offset(points, angle, spacing, range_max=15.0, bin_width=0.1,
               window=0.75):
    """
    Posição das fileiras na normal da direção angle (frame do robô).

    points (N, 2) no base_link; angle: direção das fileiras no robô (rad).
    Projeta os pontos a até range_max m na normal (-sin, cos) e procura o
    pico do histograma módulo spacing. Devolve (fase em [0, spacing),
    concentração): a concentração é a fração de pontos a até window m do
    pico sobre a de pontos espalhados por igual (1 = sem fileiras).
    """
    r = np.hypot(points[:, 0], points[:, 1])
    points = points[r < range_max]
    if len(points) == 0:
        return 0.0, 0.0
    d = -points[:, 0] * math.sin(angle) + points[:, 1] * math.cos(angle)
    phase = np.mod(d, spacing)
    bins = max(int(round(spacing / bin_width)), 4)
    counts = np.bincount(np.minimum((phase / spacing * bins).astype(np.int64),
                                    bins - 1), minlength=bins)
    # Pico do histograma suavizado (janela de ~0,3 m) e depois a média dos
    # pontos a até window m dele, recentrando (média móvel) três vezes.
    half = max(int(round(0.15 / (spacing / bins))), 1)
    kernel = np.ones(2 * half + 1)
    padded = np.concatenate([counts[-half:], counts, counts[:half]])
    smooth = np.convolve(padded, kernel, mode='valid')
    center = (int(smooth.argmax()) + 0.5) * spacing / bins
    for _ in range(3):
        offset = wrap_spacing(phase - center, spacing)
        near = np.abs(offset) < window
        if not near.any():
            return 0.0, 0.0
        center = (center + float(offset[near].mean())) % spacing
    concentration = float(near.mean()) / min(2 * window / spacing, 1.0)
    return center, concentration


def wrap_spacing(value, spacing):
    """Valor em [-spacing/2, spacing/2)."""
    return (value + spacing / 2) % spacing - spacing / 2


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
        # Posição lateral pelas fileiras (lateral_correction:=rows).
        self.declare_parameter('lateral', False)
        self.declare_parameter('row_spacing', 7.0)     # m entre fileiras
        self.declare_parameter('tree_spacing', 7.0)    # m entre árvores
        self.declare_parameter('lateral_range', 15.0)
        self.declare_parameter('lateral_concentration', 2.0)
        self.declare_parameter('max_offset', 1.0)      # m
        # Medido no banco de 2 voltas do olival (pose verdadeira): 0,15 m
        # rms por nuvem, mas com erros que mudam devagar ao longo da
        # entrelinha (copas e árvores jovens ou faltando), de +-0,2 m.
        self.declare_parameter('lateral_std', 0.2)     # m por medida
        # A referência das linhas no odom é a média dos primeiros
        # reference_distance m andados (o EKF ainda quase não derivou):
        # medida num ponto só, ela herdava o erro local de 0,1 a 0,4 m.
        self.declare_parameter('reference_distance', 20.0)
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
        self.lateral = p('lateral').value
        # Espaçamento das linhas na normal: k par anda ao longo das
        # fileiras (linhas = fileiras), k ímpar ao longo das colunas.
        self.spacing = (p('row_spacing').value, p('tree_spacing').value)
        self.lateral_range = p('lateral_range').value
        self.min_concentration = p('lateral_concentration').value
        self.max_offset = p('max_offset').value
        self.lateral_variance = p('lateral_std').value ** 2
        self.reference_distance = p('reference_distance').value
        self.reference_start = None
        # Fase das linhas no odom (k par, k ímpar), medida na partida.
        self.reference = [None, None]
        self.reference_samples = ([], [])
        self.lateral_sent = self.lateral_rejected = 0
        self.position = None        # (x, y) da última odometria

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
        self.position_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/row_heading/position', 10)
        self.create_subscription(Odometry, p('odom_topic').value,
                                 self.odom_callback, 20)
        self.create_subscription(PointCloud2, p('cloud_topic').value,
                                 self.cloud_callback, qos_profile_sensor_data)
        self.create_timer(60.0, self.log_stats)

    def odom_callback(self, message):
        self.odom = (yaw_of(message.pose.pose.orientation),
                     message.twist.twist.angular.z)
        position = message.pose.pose.position
        self.position = (position.x, position.y)
        self.history.append((
            Time.from_msg(message.header.stamp).nanoseconds * 1e-9,
            self.odom[0], position.x, position.y))

    def log_stats(self):
        if self.sent or self.rejected:
            self.get_logger().info(
                f'Rumo pelas fileiras: {self.sent} medidas enviadas ao EKF, '
                f'{self.rejected} recusadas no último minuto.')
        if self.lateral_sent or self.lateral_rejected:
            self.get_logger().info(
                f'Posição lateral pelas fileiras: {self.lateral_sent} '
                f'medidas enviadas ao EKF, {self.lateral_rejected} recusadas '
                'no último minuto.')
        self.sent = self.rejected = 0
        self.lateral_sent = self.lateral_rejected = 0

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

    def odom_at(self, t):
        """(yaw, x, y) do EKF no instante t (a odometria mais perto, 0,2 s)."""
        if not self.history:
            return None
        best = min(self.history, key=lambda item: abs(item[0] - t))
        if abs(best[0] - t) > 0.2:
            return None
        return best[1:]

    def cloud_callback(self, cloud):
        now = Time.from_msg(cloud.header.stamp).nanoseconds * 1e-9
        if self.last_time is not None and now - self.last_time < self.period:
            return
        if self.odom is None or abs(self.odom[1]) > self.max_yaw_rate:
            return
        self.last_time = now
        pose_then = self.odom_at(now)
        if pose_then is None:
            self.get_logger().warn('Sem odometria no instante da nuvem.',
                                   throttle_duration_sec=30.0)
            return
        yaw_then = pose_then[0]
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
        if self.lateral and None in self.reference:
            # Referência das linhas: junta os primeiros metros e segue
            # medindo o yaw.
            self.calibrate_lateral(obstacles, pose_then)
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
        if self.lateral and None not in self.reference:
            self.measure_lateral(obstacles, angle, k, pose_then)

    def normal(self, parity):
        """Devolve a normal (no odom) das linhas de paridade 0 (fileiras) ou 1."""
        direction = self.azimuth + parity * QUARTER
        return -math.sin(direction), math.cos(direction)

    def rows_in_odom(self, obstacles, angle, k, pose):
        """
        Fase das linhas no odom, na normal canônica da paridade k % 2.

        angle: direção das linhas no frame do robô; pose: (yaw, x, y) do EKF
        no instante da nuvem. Devolve (fase, concentração).
        """
        parity = k % 2
        spacing = self.spacing[parity]
        offset, concentration = row_offset(
            obstacles, angle, spacing, self.lateral_range)
        # k >= 2: a direção das linhas no robô é a canônica + 180 graus, e
        # a normal do robô aponta ao contrário da canônica.
        sign = 1.0 if k % 4 < 2 else -1.0
        nx, ny = self.normal(parity)
        robot = nx * pose[1] + ny * pose[2]
        return (robot + sign * offset) % spacing, concentration

    def calibrate_lateral(self, obstacles, pose):
        # Nos primeiros reference_distance m (o EKF ainda quase não
        # derivou), a fase das linhas nas duas direções da grade, em média,
        # vira a referência no odom.
        if self.reference_start is None:
            self.reference_start = pose[1:]
        for parity in (0, 1):
            # Direção canônica das linhas (k = parity) no frame do robô.
            predicted = wrap(self.azimuth + parity * QUARTER - pose[0])
            angle, quality, edge = heading_from_points(
                obstacles, predicted, self.gate, bin_width=self.bin_width,
                range_max=self.range_max)
            if edge or quality < self.min_quality:
                continue
            phase, concentration = self.rows_in_odom(
                obstacles, angle, parity, pose)
            if concentration >= self.min_concentration:
                self.reference_samples[parity].append(phase)
        travelled = math.hypot(pose[1] - self.reference_start[0],
                               pose[2] - self.reference_start[1])
        if (travelled < self.reference_distance
                or min(len(s) for s in self.reference_samples) < (
                    self.calibration_samples)):
            return
        for parity in (0, 1):
            spacing = self.spacing[parity]
            angles = [2 * math.pi * v / spacing
                      for v in self.reference_samples[parity]]
            mean = math.atan2(np.mean(np.sin(angles)),
                              np.mean(np.cos(angles)))
            self.reference[parity] = (mean / (2 * math.pi) * spacing) % spacing
        self.get_logger().info(
            f'Linhas no odom a {self.reference[0]:.2f} m (fileiras) e '
            f'{self.reference[1]:.2f} m (colunas) na normal '
            f'({len(self.reference_samples[0])} e '
            f'{len(self.reference_samples[1])} nuvens em '
            f'{travelled:.0f} m); a posição lateral pelas fileiras passa a '
            'corrigir o EKF.')

    def measure_lateral(self, obstacles, angle, k, pose):
        parity = k % 2
        spacing = self.spacing[parity]
        phase, concentration = self.rows_in_odom(obstacles, angle, k, pose)
        error = wrap_spacing(self.reference[parity] - phase, spacing)
        if (concentration < self.min_concentration
                or abs(error) > self.max_offset):
            self.lateral_rejected += 1
            return
        # O EKF está error m atrasado na normal: soma à posição atual (o
        # deslocamento desde a nuvem não muda o erro).
        nx, ny = self.normal(parity)
        x, y = self.position
        self.publish_position(x + error * nx, y + error * ny, nx, ny)
        self.lateral_sent += 1

    def publish_position(self, x, y, nx, ny):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.w = 1.0
        # Variância lateral_std² na normal e 1e6 ao longo das linhas.
        along = 1e6
        across = self.lateral_variance
        covariance = [0.0] * 36
        covariance[0] = across * nx * nx + along * ny * ny
        covariance[1] = covariance[6] = (across - along) * nx * ny
        covariance[7] = across * ny * ny + along * nx * nx
        for i in range(2, 6):
            covariance[i * 7] = 1e6     # z, roll, pitch, yaw: não usados
        message.pose.covariance = covariance
        self.position_publisher.publish(message)

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
