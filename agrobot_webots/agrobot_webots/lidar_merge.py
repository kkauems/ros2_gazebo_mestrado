#!/usr/bin/env python3
"""
Junta o scan do LiDAR de segurança à nuvem do LiDAR 3D para o RTAB-Map.

O RTAB-Map assina uma nuvem só e usa a mesma para o ICP e para a grade. Os
dois LiDARs servem para coisas diferentes:

- ICP: a nuvem 3D (chão, copas, troncos) trava x, y e yaw.
- Grade: só o scan de segurança, como antes do LiDAR 3D. O LiDAR 3D fica a
  ~1 m do chão e só vê um obstáculo baixo (um colmo, um tronco, um monte de
  galhos) numa faixa a ~2,6 a 4 m do robô; os raios dele até o chão passam
  por cima das plantas e, na grade 2D, apagavam o que a passada anterior
  tinha marcado. E com o relevo (camalhões, encosta), o chão a 5 ou 6 m
  caía na faixa de obstáculo e virava obstáculo falso no meio da entrelinha.

Para a grade usar só o scan, os pontos do LiDAR 3D entre drop_z_min e
drop_z_max (no base_frame) saem da nuvem: abaixo de drop_z_min a grade os
ignora (Grid/MinGroundHeight) e acima de drop_z_max também
(Grid/MaxObstacleHeight). O scan entra a 0,15 m do chão (obstáculo), e cada
raio sem retorno vira um ponto de chão a fill_range m (como o
Grid/Scan2dUnknownSpaceFilled do scan 2D), para marcar o espaço livre.

O scan de segurança é um plano: com o relevo, ele bate no chão a alguns
metros. Um ponto do scan sem nada mais alto do LiDAR 3D por perto é chão e
sai (terrain_hits).

Cada nuvem nova do LiDAR 3D sai com o último scan junto, levado ao frame do
LiDAR 3D pelo TF no instante de cada um (com o odom como frame fixo): o
scan é de até ~0,16 s antes, e girando a 1 rad/s isso daria ~9 graus de erro.

Tópicos:
    /scan              (sensor_msgs/LaserScan)    entrada
    /lidar3d/points    (sensor_msgs/PointCloud2)  entrada
    /lidar3d/merged    (sensor_msgs/PointCloud2)  saída, x y z float32
"""

import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
import tf2_ros


# scan_in_cloud_frame: o TF do instante da nuvem ainda não chegou.
WAIT = object()


def scan_points(scan, fill_range=None, fill_z=0.0):
    """
    Pontos (N, 3) do LaserScan no frame dele e os de preenchimento.

    Devolve (retornos, preenchimento): cada raio sem retorno (+inf) vira um
    ponto a fill_range m, na altura fill_z (abaixo do plano do scan: chão);
    sem fill_range, nenhum.
    """
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
    valid = (np.isfinite(ranges) & (ranges >= scan.range_min)
             & (ranges <= scan.range_max))
    r, a = ranges[valid], angles[valid]
    hits = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=-1)
    if fill_range is None:
        return hits, np.zeros((0, 3))
    a = angles[np.isposinf(ranges)]
    fill = np.stack([fill_range * np.cos(a), fill_range * np.sin(a),
                     np.full_like(a, fill_z)], axis=-1)
    return hits, fill


def terrain_hits(scan, cloud, radius, margin, seen_from=None):
    """
    Máscara dos pontos do scan que são o chão subindo, pela nuvem 3D.

    scan e cloud em (N, 3) no frame do robô (z para cima). O LiDAR de
    segurança é um plano: com o relevo, ele bate no chão a alguns metros, e
    isso parece obstáculo. Um obstáculo de verdade (planta, tronco, pedra,
    pessoa) tem pontos do LiDAR 3D acima da altura do scan; o chão, não. O
    ponto do scan é chão se há pontos da nuvem a até radius dele (em x, y) e
    nenhum passa de z do scan + margin. Sem pontos da nuvem por perto, fica
    como obstáculo se estiver a menos de seen_from (m, em x, y, da origem do
    frame) e sai se estiver além: o LiDAR 3D não vê o chão a menos de ~3 m,
    mas além disso um obstáculo de verdade aparece na nuvem (alto, ele não
    some atrás das plantas como um galho num camalhão). Sem seen_from, todo
    ponto sem nuvem por perto fica.
    """
    ground = np.zeros(len(scan), dtype=bool)
    if not len(scan) or not len(cloud):
        return ground
    reach = np.abs(scan[:, :2]).max() + radius
    near = cloud[(np.abs(cloud[:, 0]) <= reach) & (np.abs(cloud[:, 1]) <= reach)]
    for i, point in enumerate(scan):
        close = near[np.hypot(near[:, 0] - point[0], near[:, 1] - point[1])
                     <= radius]
        if len(close):
            ground[i] = close[:, 2].max() <= point[2] + margin
        elif seen_from is not None:
            ground[i] = np.hypot(point[0], point[1]) > seen_from
    return ground


def matrix_from_transform(transform):
    """Rotação 3x3 e translação de um geometry_msgs/Transform."""
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    t = transform.translation
    return rotation, np.array([t.x, t.y, t.z])


def xyz_fields(cloud):
    """Confere se a nuvem é x, y, z float32 contíguos (a do driver)."""
    names = [(f.name, f.offset, f.datatype) for f in cloud.fields]
    return (cloud.point_step == 12 and names[:3] == [
        ('x', 0, 7), ('y', 4, 7), ('z', 8, 7)])


class LidarMerge(Node):
    """Nuvem do LiDAR 3D mais o último scan de segurança."""

    def __init__(self):
        super().__init__('lidar_merge')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/lidar3d/points')
        self.declare_parameter('output_topic', '/lidar3d/merged')
        self.declare_parameter('fixed_frame', 'odom')
        # Scan mais velho que isto (s) não entra.
        self.declare_parameter('max_scan_age', 0.5)
        # Faixa (z no base_frame) de onde os pontos do LiDAR 3D saem: a que
        # a grade do RTAB-Map usa (Grid/MinGroundHeight a
        # Grid/MaxObstacleHeight).
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('drop_z_min', -0.20)
        self.declare_parameter('drop_z_max', 0.10)
        self.base_frame = self.get_parameter('base_frame').value
        self.drop_z = (self.get_parameter('drop_z_min').value,
                       self.get_parameter('drop_z_max').value)
        self.cloud_to_base = None
        # Raio sem retorno: ponto de chão a esta distância (m) e altura (m,
        # no frame do scan; o scan fica 0,15 m acima do chão).
        self.declare_parameter('fill_range', 5.0)
        self.declare_parameter('fill_z', -0.06)
        self.fill_range = self.get_parameter('fill_range').value
        self.fill_z = self.get_parameter('fill_z').value
        # Ponto do scan sem nada do LiDAR 3D mais alto que ele (+ margem) a
        # até este raio: é o chão subindo (relevo), não obstáculo.
        self.declare_parameter('terrain_radius', 0.15)
        self.declare_parameter('terrain_margin', 0.05)
        self.terrain_radius = self.get_parameter('terrain_radius').value
        self.terrain_margin = self.get_parameter('terrain_margin').value
        # Até aqui (m) o LiDAR 3D não vê o que está baixo: o scan vale
        # sozinho. Além, um ponto do scan precisa de pontos da nuvem perto.
        self.declare_parameter('blind_range', 3.0)
        self.blind_range = self.get_parameter('blind_range').value
        self.dropped = 0
        self.kept = 0
        self.create_timer(60.0, self.log_terrain)
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.max_age = self.get_parameter('max_scan_age').value
        self.scan = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self.cloud_callback, qos_profile_sensor_data)
        self.warned = False
        # Nuvens esperando o TF do instante delas.
        self.pending = []
        self.max_wait = 0.5
        self.create_timer(0.02, self.process_pending)

    def scan_callback(self, message):
        self.scan = message

    def scan_in_cloud_frame(self, cloud):
        scan = self.scan
        if scan is None:
            return None
        age = (Time.from_msg(cloud.header.stamp)
               - Time.from_msg(scan.header.stamp)).nanoseconds / 1e9
        if abs(age) > self.max_age:
            return None
        try:
            transform = self.tf_buffer.lookup_transform_full(
                cloud.header.frame_id, Time.from_msg(cloud.header.stamp),
                scan.header.frame_id, Time.from_msg(scan.header.stamp),
                self.fixed_frame)
        except tf2_ros.TransformException as error:
            # O TF do odom (EKF) chega um pouco depois da nuvem: espera.
            waited = (self.get_clock().now()
                      - Time.from_msg(cloud.header.stamp)).nanoseconds / 1e9
            if waited < self.max_wait:
                return WAIT
            if not self.warned:
                self.get_logger().warn(
                    f'Sem TF para juntar o scan à nuvem ({error}); a nuvem '
                    'sai sem o scan até o TF chegar.')
                self.warned = True
            return None
        rotation, translation = matrix_from_transform(transform.transform)
        hits, fill = scan_points(scan, self.fill_range, self.fill_z)
        return (hits @ rotation.T + translation,
                fill @ rotation.T + translation)

    def cloud_in_base(self, cloud):
        """Rotação e translação do frame da nuvem para o base_frame."""
        if self.cloud_to_base is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, cloud.header.frame_id, Time())
            except tf2_ros.TransformException:
                return None
            rotation, translation = matrix_from_transform(
                transform.transform)
            # O LiDAR é fixo no robô: basta uma vez.
            self.cloud_to_base = (rotation, translation)
        return self.cloud_to_base

    def log_terrain(self):
        total = self.dropped + self.kept
        if total:
            self.get_logger().info(
                f'Scan de segurança na grade: {self.dropped} de {total} '
                'pontos eram o chão (relevo) e ficaram de fora.')
        self.dropped = self.kept = 0

    def cloud_callback(self, cloud):
        self.pending.append(cloud)
        # Atrasada demais (o TF não veio): a mais velha sai sem o scan.
        while len(self.pending) > 5:
            self.publish(self.pending.pop(0), use_scan=False)

    def process_pending(self):
        while self.pending:
            if not self.publish(self.pending[0]):
                return
            self.pending.pop(0)

    def publish(self, cloud, use_scan=True):
        """Publica a nuvem com o scan; False se ainda falta o TF."""
        if not xyz_fields(cloud):
            self.get_logger().error(
                'A nuvem precisa ser x, y, z float32 (point_step 12).',
                throttle_duration_sec=10.0)
            return True
        base = self.cloud_in_base(cloud)
        if base is None:
            return False
        extra = self.scan_in_cloud_frame(cloud) if use_scan else None
        if extra is WAIT:
            return False
        points = np.frombuffer(bytes(cloud.data), dtype=np.float32).reshape(
            -1, 3)
        rotation, translation = base
        in_base = points.astype(np.float64) @ rotation.T + translation
        if extra is not None:
            hits, fill = extra
            # Distâncias a partir do LiDAR 3D: no frame dele, só deslocado.
            ground = terrain_hits(
                hits @ rotation.T, points.astype(np.float64) @ rotation.T,
                self.terrain_radius, self.terrain_margin, self.blind_range)
            hits = hits[~ground]
            self.dropped += int(ground.sum())
            self.kept += len(hits)
            extra = np.concatenate([hits, fill])
        z = in_base[:, 2]
        points = points[(z <= self.drop_z[0]) | (z > self.drop_z[1])]
        message = PointCloud2()
        message.header = cloud.header
        message.height = 1
        message.fields = cloud.fields[:3]
        message.is_bigendian = False
        message.point_step = 12
        message.is_dense = True
        data = points.tobytes()
        count = len(points)
        if extra is not None and len(extra):
            data += extra.astype(np.float32).tobytes()
            count += len(extra)
        message.width = count
        message.row_step = 12 * count
        message.data = array.array('B', data)
        self.publisher.publish(message)
        return True


def main():
    rclpy.init()
    node = LidarMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
