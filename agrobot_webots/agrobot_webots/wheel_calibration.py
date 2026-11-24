#!/usr/bin/env python3
r"""
Calibração das rodas e do escorregamento lateral numa rodada de referência.

O que mede (só nos trechos retos, com o robô andando):
- raio efetivo: distância horizontal verdadeira / distância das rodas. É o
  wheelRadiusScale do driver (wheel_radius_scale no launch). O pneu real
  muda de raio com a carga, a pressão e o solo, e no simulador o ruído dos
  encoders sorteia esse erro por semente (sensor_noise.py, ~1%);
- offset de roll da IMU: o roll medido inclui a inclinação fixa do offset
  do acelerômetro. Na ida e na volta pela mesma entrelinha a encosta
  inverte o roll e o offset não, então o offset é a média das duas médias
  (não usa a pose verdadeira);
- ganho do escorregamento encosta abaixo: velocidade lateral verdadeira =
  ganho * sin(roll da IMU - offset) * |velocidade das rodas|, por mínimos
  quadrados. É o modelo do slope_slip.py (lateral_correction:=imu).

A referência é a pose verdadeira do Webots (/ground_truth/odom): no campo
seria uma trena, um GNSS RTK ou marcos conhecidos ao longo da entrelinha.
Rode com wheel_radius_scale:=1.0 (sem calibração anterior):

    ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt \\
        ekf:=true vslam:=true noise_seed:=101 wheel_radius_scale:=1.0 \\
        lateral_correction:=none waypoints:=calibracao_rodas_olival \\
        wheel_calibration:=true

O resultado sai no log e em dados/calibracao/<mundo>_semente<N>.yaml, no
formato de config/wheel_calibration.yaml.

Tópicos:
    /wheel/odom         (nav_msgs/Odometry)  rodas, direto do driver
    /ground_truth/odom  (nav_msgs/Odometry)  pose verdadeira (Webots)
    /imu/data           (sensor_msgs/Imu)    roll medido
"""

import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def roll_pitch_yaw(q):
    """Ângulos (rad) de um quaternion (x, y, z, w)."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rotate(q, v):
    """Vetor v girado pelo quaternion q (x, y, z, w)."""
    x, y, z, w = q
    u = np.array([x, y, z])
    v = np.asarray(v, dtype=float)
    return (2 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v
            + 2 * w * np.cross(u, v))


def horizontal_velocity(q, body_velocity):
    """
    Velocidade para a frente e lateral no plano horizontal (m/s).

    body_velocity está no frame do robô (como no /ground_truth/odom); o
    EKF é 2D e usa a velocidade projetada no chão, no rumo do robô.
    """
    vw = rotate(q, body_velocity)
    yaw = roll_pitch_yaw(q)[2]
    forward = math.cos(yaw) * vw[0] + math.sin(yaw) * vw[1]
    lateral = -math.sin(yaw) * vw[0] + math.cos(yaw) * vw[1]
    return forward, lateral


def roll_offset(roll, heading):
    """
    Offset fixo do roll medido: média das médias em sentidos opostos.

    heading: rumo de cada amostra em quartos de volta (0 a 3). Numa ida e
    volta pela mesma entrelinha, a encosta inverte o sinal do roll e o
    offset não. Sem sentidos opostos, devolve None.
    """
    roll = np.asarray(roll)
    heading = np.asarray(heading)
    pairs = []
    for k in (0, 1):
        a, b = roll[heading == k], roll[heading == k + 2]
        if len(a) >= 30 and len(b) >= 30:
            pairs.append(((a.mean() + b.mean()) / 2, min(len(a), len(b))))
    if not pairs:
        return None
    weights = np.array([n for _, n in pairs], dtype=float)
    return float(np.dot([v for v, _ in pairs], weights) / weights.sum())


def slip_gain(lateral, roll, speed, offset):
    """Ganho g de lateral = g * sin(roll - offset) * |speed| (mín. quadr.)."""
    x = np.sin(np.asarray(roll) - offset) * np.abs(speed)
    denominator = float(np.dot(x, x))
    if denominator <= 0.0:
        return 0.0, 0.0
    gain = float(np.dot(x, lateral) / denominator)
    residual = np.asarray(lateral) - gain * x
    explained = 1.0 - float(np.var(residual)) / max(
        float(np.var(lateral)), 1e-12)
    return gain, explained


class WheelCalibration(Node):
    """Junta os trechos retos e calcula a calibração."""

    def __init__(self):
        super().__init__('wheel_calibration')
        self.declare_parameter('output_file', '')
        self.declare_parameter('min_speed', 0.15)       # m/s
        self.declare_parameter('max_yaw_rate', 0.05)    # rad/s
        self.declare_parameter('period', 20.0)          # s entre os relatórios
        self.output_file = self.get_parameter('output_file').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.truth = None
        self.imu_roll = None
        self.last_time = None
        # Amostras dos trechos retos.
        self.samples = {k: [] for k in (
            'dt', 'wheel', 'forward', 'lateral', 'roll', 'roll_true',
            'heading')}
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 self.truth_callback, 50)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)
        self.create_subscription(Odometry, '/wheel/odom',
                                 self.wheel_callback, 50)
        self.create_timer(self.get_parameter('period').value, self.report)

    def truth_callback(self, message):
        self.truth = message

    def imu_callback(self, message):
        o = message.orientation
        self.imu_roll = roll_pitch_yaw((o.x, o.y, o.z, o.w))[0]

    def wheel_callback(self, message):
        t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        dt = None if self.last_time is None else t - self.last_time
        self.last_time = t
        if dt is None or not 0.0 < dt < 0.2:
            return
        if self.truth is None or self.imu_roll is None:
            return
        speed = message.twist.twist.linear.x
        truth = self.truth.twist.twist
        if (abs(speed) < self.min_speed
                or abs(truth.angular.z) > self.max_yaw_rate):
            return
        o = self.truth.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        forward, lateral = horizontal_velocity(
            q, (truth.linear.x, truth.linear.y, truth.linear.z))
        roll_true, _, yaw = roll_pitch_yaw(q)
        s = self.samples
        s['dt'].append(dt)
        s['wheel'].append(speed)
        s['forward'].append(forward)
        s['lateral'].append(lateral)
        s['roll'].append(self.imu_roll)
        s['roll_true'].append(roll_true)
        s['heading'].append(round(yaw / (math.pi / 2)) % 4)

    def result(self):
        s = {k: np.asarray(v) for k, v in self.samples.items()}
        if len(s['dt']) < 100:
            return None
        wheel = float(np.sum(np.abs(s['wheel']) * s['dt']))
        truth = float(np.sum(np.abs(s['forward']) * s['dt']))
        offset = roll_offset(s['roll'], s['heading'])
        out = {
            'distance_m': truth,
            'samples': int(len(s['dt'])),
            'wheel_radius_scale': truth / wheel,
            'roll_offset_deg': None,
            'slope_slip_gain': None,
            'slope_slip_explained': None,
            # Referência: offset verdadeiro (roll medido - roll real).
            'roll_offset_true_deg': math.degrees(
                float(np.mean(s['roll'] - s['roll_true']))),
        }
        if offset is not None:
            gain, explained = slip_gain(
                s['lateral'], s['roll'], s['wheel'] * out['wheel_radius_scale'],
                offset)
            out['roll_offset_deg'] = math.degrees(offset)
            out['slope_slip_gain'] = gain
            out['slope_slip_explained'] = explained
        return out

    def report(self):
        out = self.result()
        if out is None:
            return
        text = (f'Calibração: {out["distance_m"]:.1f} m em trechos retos, '
                f'raio efetivo x {out["wheel_radius_scale"]:.4f}')
        if out['roll_offset_deg'] is not None:
            text += (f', offset do roll {out["roll_offset_deg"]:.2f} graus '
                     f'(verdadeiro {out["roll_offset_true_deg"]:.2f}), '
                     f'ganho do escorregamento {out["slope_slip_gain"]:.3f} '
                     f'(explica {100 * out["slope_slip_explained"]:.0f}% da '
                     'velocidade lateral)')
        else:
            text += ' (falta andar nos dois sentidos para o roll)'
        self.get_logger().info(text)
        if self.output_file:
            self.write(out)

    def write(self, out):
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        lines = [
            '# Gerado pelo wheel_calibration.py; copie os valores para',
            '# config/wheel_calibration.yaml.',
            f'distance_m: {out["distance_m"]:.1f}',
            f'samples: {out["samples"]}',
            f'wheel_radius_scale: {out["wheel_radius_scale"]:.4f}',
        ]
        if out['roll_offset_deg'] is not None:
            lines += [
                f'roll_offset_deg: {out["roll_offset_deg"]:.3f}',
                f'slope_slip_gain: {out["slope_slip_gain"]:.4f}',
                f'slope_slip_explained: {out["slope_slip_explained"]:.3f}',
            ]
        lines.append(
            f'roll_offset_true_deg: {out["roll_offset_true_deg"]:.3f}')
        with open(self.output_file, 'w') as handle:
            handle.write('\n'.join(lines) + '\n')


def main():
    rclpy.init()
    node = WheelCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
