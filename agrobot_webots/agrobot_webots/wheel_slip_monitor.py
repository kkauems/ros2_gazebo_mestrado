#!/usr/bin/env python3
"""
Detecta a patinagem das rodas comparando-as com a odometria visual.

Quando o robô fica preso (contra uma pedra, por exemplo) e as rodas continuam
girando, a odometria das rodas diz que ele anda e o EKF segue as rodas: a
pose estimada vai embora com o robô parado. O contrário também acontece: o
robô arrastado sem as rodas girarem. Nem a IMU pega isso (parado e andando a
velocidade constante dão a mesma aceleração, zero).

Este nó fica entre o driver e o EKF. Ele recebe as rodas (/wheel/odom) e a
odometria visual da câmera RGB-D (/vo/odom, do rgbd_odometry do RTAB-Map) e
republica as rodas em /wheel/odom_checked, que é o que o EKF lê:
- normal: a mensagem das rodas passa sem mudança;
- patinando (nas leituras válidas da câmera dos últimos detect_time
  segundos, quase todas discordam da média das rodas entre os mesmos dois
  quadros por mais de speed_tolerance + relative_tolerance * |v|): vx e vy
  passam a ser a mediana recente da câmera, com a variância dela, até as
  duas voltarem a concordar durante release_time. Se a câmera se perde por
  alguns quadros no meio (o robô sacudindo contra a pedra), a última
  velocidade dela fica valendo, com a incerteza crescendo, por até
  hold_time. Leituras impossíveis da câmera (rápidas demais, ou logo depois
  de ela se perder) não contam; os detalhes estão no slip_detector.py;
- sem odometria visual (câmera perdida por mais de hold_time, nó
  desligado): a mensagem das rodas passa sem mudança, como sem este nó.

Só a velocidade linear é trocada. O yaw do EKF vem da IMU, que não patina.

Tópicos:
    /wheel/odom          (nav_msgs/Odometry)  entrada, do driver
    /vo/odom             (nav_msgs/Odometry)  entrada, do rgbd_odometry
    /wheel/odom_checked  (nav_msgs/Odometry)  saída, para o EKF
    /wheel/slip          (std_msgs/Bool)      saída, true enquanto patina
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool

from agrobot_webots.slip_detector import SlipDetector


def stamp_seconds(message):
    return Time.from_msg(message.header.stamp).nanoseconds * 1e-9


class WheelSlipMonitor(Node):
    """Republica as rodas para o EKF, trocando-as pela câmera na patinagem."""

    def __init__(self):
        super().__init__('wheel_slip_monitor')
        defaults = {
            'speed_tolerance': 0.10,        # m/s
            'relative_tolerance': 0.30,     # fração de |v|
            'detect_time': 1.5,             # s de janela para entrar
            'release_time': 1.0,            # s de janela para sair
            'min_samples': 8,               # leituras válidas na janela
            'min_fraction': 0.8,            # fração que discorda/concorda
            'max_reference_speed': 0.8,     # m/s; o robô anda até 0,5
            'skip_after_loss': 3,           # leituras ignoradas após perda
            # Patinando, quanto tempo segurar a velocidade da câmera quando
            # ela se perde (robô sacudindo contra a pedra).
            'hold_time': 3.0,               # s
            # Piso da variância de vx e vy da câmera: (5 cm/s)^2. O
            # rgbd_odometry às vezes dá variâncias quase nulas, e o EKF
            # passaria a ignorar todo o resto.
            'min_reference_variance': 0.0025,
            'hold_std_growth': 0.1,         # (m/s) por s sem câmera
        }
        values = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        self.detector = SlipDetector(**values)
        self.reported_rejections = 0

        self.output = self.create_publisher(
            Odometry, '/wheel/odom_checked', 10)
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.slip_publisher = self.create_publisher(
            Bool, '/wheel/slip', latched)
        self.slip_publisher.publish(Bool(data=False))
        self.create_subscription(
            Odometry, '/vo/odom', self.reference_callback, 10)
        self.create_subscription(
            Odometry, '/wheel/odom', self.wheel_callback, 10)

    def reference_callback(self, message):
        now = stamp_seconds(message)
        changed, reason = self.detector.add_reference(
            now, message.twist.twist.linear.x, message.twist.twist.linear.y,
            message.twist.covariance[0], message.twist.covariance[7])
        if changed:
            self.report(now, reason)
        rejected = self.detector.rejected
        total = rejected['rapida demais'] + rejected['logo apos perda']
        if total >= self.reported_rejections + 10:
            self.reported_rejections = total
            self.get_logger().info(
                'Leituras da odometria visual descartadas até agora: '
                + ', '.join(f'{n} {k}' for k, n in rejected.items()))

    def wheel_callback(self, message):
        now = stamp_seconds(message)
        self.detector.add_wheel(
            now, message.twist.twist.linear.x, message.twist.twist.linear.y)
        was_slipping = self.detector.slipping
        replacement = self.detector.wheel_output(now)
        if was_slipping and not self.detector.slipping:
            self.report(now, f'sem odometria visual válida há mais de '
                             f'{self.detector.hold_time:.0f} s')
        if replacement is not None:
            vx, vy, var_x, var_y = replacement
            message.twist.twist.linear.x = vx
            message.twist.twist.linear.y = vy
            message.twist.covariance[0] = var_x
            message.twist.covariance[7] = var_y
        self.output.publish(message)

    def report(self, now, reason):
        detector = self.detector
        if detector.slipping:
            self.get_logger().warn(
                f'Patinagem {detector.slip_count} em t = {now:.1f} s '
                f'({reason}): o EKF passa a usar a velocidade da câmera.')
        else:
            duration = now - (detector.slip_started or now)
            self.get_logger().info(
                f'Fim da patinagem em t = {now:.1f} s, {duration:.1f} s '
                f'depois ({reason}): o EKF volta às rodas.')
        self.slip_publisher.publish(Bool(data=detector.slipping))


def main(args=None):
    rclpy.init(args=args)
    node = WheelSlipMonitor()
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
