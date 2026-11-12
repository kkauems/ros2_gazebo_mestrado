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
- patinando (a velocidade da câmera e a média das rodas entre os mesmos dois
  quadros discordam por mais de speed_tolerance + relative_tolerance * |v|
  durante detect_time): vx e vy passam a ser os da câmera, com a variância
  dela, até as duas voltarem a concordar durante release_time. Se a câmera
  se perde por alguns quadros no meio (o robô sacudindo contra a pedra), a
  última velocidade dela fica valendo, com a incerteza crescendo, por até
  hold_time;
- sem odometria visual (câmera perdida por mais de hold_time, nó
  desligado): a mensagem das rodas passa sem mudança, como sem este nó.

Só a velocidade linear é trocada. O yaw do EKF vem da IMU, que não patina.

Tópicos:
    /wheel/odom          (nav_msgs/Odometry)  entrada, do driver
    /vo/odom             (nav_msgs/Odometry)  entrada, do rgbd_odometry
    /wheel/odom_checked  (nav_msgs/Odometry)  saída, para o EKF
    /wheel/slip          (std_msgs/Bool)      saída, true enquanto patina
"""

from collections import deque
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool

# O rtabmap marca a odometria perdida com variância 9999 (ou a publica zerada,
# com variância 0). Acima disto, a medida não vale.
MAX_REFERENCE_VARIANCE = 1.0
# Quanto das rodas guardar para comparar com a câmera (s).
WHEEL_HISTORY = 2.0


def stamp_seconds(message):
    return Time.from_msg(message.header.stamp).nanoseconds * 1e-9


class WheelSlipMonitor(Node):
    """Republica as rodas para o EKF, trocando-as pela câmera na patinagem."""

    def __init__(self):
        super().__init__('wheel_slip_monitor')
        self.speed_tolerance = self.declare_parameter(
            'speed_tolerance', 0.10).value          # m/s
        self.relative_tolerance = self.declare_parameter(
            'relative_tolerance', 0.30).value       # fração de |v|
        self.detect_time = self.declare_parameter(
            'detect_time', 0.5).value               # s discordando
        self.release_time = self.declare_parameter(
            'release_time', 1.0).value              # s concordando
        # Patinando, quanto tempo segurar a última velocidade da câmera
        # quando ela se perde por alguns quadros (robô sacudindo contra a
        # pedra). Depois disso, as rodas voltam.
        self.hold_time = self.declare_parameter(
            'hold_time', 3.0).value                 # s
        # Piso da variância de vx e vy da câmera quando ela substitui as
        # rodas: (5 cm/s)^2. O rgbd_odometry às vezes dá variâncias quase
        # nulas, e o EKF passaria a ignorar todo o resto.
        self.min_reference_variance = self.declare_parameter(
            'min_reference_variance', 0.0025).value
        # Crescimento do desvio-padrão da velocidade segurada: (m/s) por s.
        self.hold_std_growth = self.declare_parameter(
            'hold_std_growth', 0.1).value

        self.wheels = deque()           # (tempo, vx, vy)
        self.reference = None           # (tempo, vx, vy, var_x, var_y)
        self.last_reference_time = None
        self.slipping = False
        self.disagree_since = None
        self.agree_since = None
        self.slip_started = None
        self.slip_count = 0

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

    def wheel_mean(self, start, end):
        """Velocidade média das rodas entre dois quadros da câmera."""
        samples = [(vx, vy) for t, vx, vy in self.wheels if start < t <= end]
        if not samples:
            return None
        return (sum(s[0] for s in samples) / len(samples),
                sum(s[1] for s in samples) / len(samples))

    def reference_callback(self, message):
        now = stamp_seconds(message)
        previous = self.last_reference_time
        self.last_reference_time = now
        var_x = message.twist.covariance[0]
        var_y = message.twist.covariance[7]
        if not 0.0 < var_x < MAX_REFERENCE_VARIANCE:
            return
        if not 0.0 < var_y < MAX_REFERENCE_VARIANCE:
            var_y = var_x
        ref_vx = message.twist.twist.linear.x
        ref_vy = message.twist.twist.linear.y
        self.reference = (now, ref_vx, ref_vy, var_x, var_y)
        if previous is None or now <= previous:
            return

        # A velocidade da câmera é o deslocamento entre dois quadros: compara
        # com a média das rodas no mesmo intervalo, e não com a última
        # mensagem delas (a câmera chega atrasada, e nas acelerações as duas
        # discordariam sem patinagem).
        wheel = self.wheel_mean(previous, now)
        if wheel is None:
            return
        difference = math.hypot(wheel[0] - ref_vx, wheel[1] - ref_vy)
        tolerance = (self.speed_tolerance + self.relative_tolerance
                     * max(abs(wheel[0]), math.hypot(ref_vx, ref_vy)))
        if difference > tolerance:
            self.agree_since = None
            if self.disagree_since is None:
                self.disagree_since = previous
            if (not self.slipping
                    and now - self.disagree_since >= self.detect_time):
                self.set_slipping(
                    True, now,
                    f'rodas {wheel[0]:+.2f} m/s, câmera {ref_vx:+.2f} m/s')
        else:
            self.disagree_since = None
            if self.agree_since is None:
                self.agree_since = previous
            if (self.slipping
                    and now - self.agree_since >= self.release_time):
                self.set_slipping(False, now, 'rodas e câmera concordam')

    def wheel_callback(self, message):
        now = stamp_seconds(message)
        self.wheels.append(
            (now, message.twist.twist.linear.x, message.twist.twist.linear.y))
        while self.wheels and self.wheels[0][0] < now - WHEEL_HISTORY:
            self.wheels.popleft()

        if self.slipping:
            reference = self.reference
            age = (now - reference[0]) if reference is not None else None
            if age is None or age > self.hold_time:
                self.set_slipping(
                    False, now,
                    f'sem odometria visual há mais de {self.hold_time:.0f} s')
            else:
                # Última velocidade da câmera, com a incerteza crescendo
                # enquanto ela não volta.
                _, ref_vx, ref_vy, ref_var_x, ref_var_y = reference
                extra = (self.hold_std_growth * max(age, 0.0)) ** 2
                message.twist.twist.linear.x = ref_vx
                message.twist.twist.linear.y = ref_vy
                message.twist.covariance[0] = max(
                    ref_var_x, self.min_reference_variance) + extra
                message.twist.covariance[7] = max(
                    ref_var_y, self.min_reference_variance) + extra
        self.output.publish(message)

    def set_slipping(self, slipping, now, reason):
        if slipping == self.slipping:
            return
        self.slipping = slipping
        self.agree_since = None
        self.disagree_since = None
        if slipping:
            self.slip_count += 1
            self.slip_started = now
            self.get_logger().warn(
                f'Patinagem {self.slip_count} em t = {now:.1f} s ({reason}): '
                'o EKF passa a usar a velocidade da câmera.')
        else:
            duration = now - (self.slip_started or now)
            self.get_logger().info(
                f'Fim da patinagem em t = {now:.1f} s, {duration:.1f} s '
                f'depois ({reason}): o EKF volta às rodas.')
        self.slip_publisher.publish(Bool(data=slipping))


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
