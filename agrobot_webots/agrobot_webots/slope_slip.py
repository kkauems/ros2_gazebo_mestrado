#!/usr/bin/env python3
"""
Escorregamento encosta abaixo pela inclinação da IMU (lateral_correction:=imu).

Andando de través numa encosta, o robô skid-steer escorrega para o lado de
baixo: os pneus, que já derrapam para a frente, perdem aderência lateral.
As rodas não veem isso (o EKF recebe vy = 0) e o erro lateral acumula: 0,3
a 0,5 m na ida de 66 m do caso 4 do olival.

O nó fica entre as rodas e o EKF e soma à velocidade lateral das rodas

    vy = ganho * sin(roll - offset) * |vx|

com o roll da IMU (roll > 0: lado esquerdo mais alto, escorrega para a
direita, ganho < 0). O ganho e o offset do roll (inclinação fixa do offset
do acelerômetro) vêm da rodada de calibração (wheel_calibration.py,
config/wheel_calibration.yaml). A variância de vy cresce com a correção:
o modelo é simples e o solo muda.

Tópicos:
    entrada  (nav_msgs/Odometry)  rodas (/wheel/odom ou /wheel/odom_checked)
    /imu/data (sensor_msgs/Imu)   roll
    saída    (nav_msgs/Odometry)  /wheel/odom_slope, para o EKF
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def roll_of(q):
    return math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))


def slope_lateral_velocity(gain, roll, offset, speed):
    """Velocidade lateral do escorregamento (m/s), no frame do robô."""
    return gain * math.sin(roll - offset) * abs(speed)


class SlopeSlip(Node):
    """Soma o escorregamento estimado à velocidade lateral das rodas."""

    def __init__(self):
        super().__init__('slope_slip')
        self.declare_parameter('input_topic', '/wheel/odom')
        self.declare_parameter('output_topic', '/wheel/odom_slope')
        self.declare_parameter('gain', 0.0)
        self.declare_parameter('roll_offset_deg', 0.0)
        # Fração da correção somada ao desvio padrão de vy.
        self.declare_parameter('uncertainty', 0.5)
        p = self.get_parameter
        self.gain = p('gain').value
        self.offset = math.radians(p('roll_offset_deg').value)
        self.uncertainty = p('uncertainty').value
        self.roll = None
        self.publisher = self.create_publisher(
            Odometry, p('output_topic').value, 10)
        # O driver publica a IMU como best effort.
        self.create_subscription(Imu, '/imu/data', self.imu_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, p('input_topic').value,
                                 self.odom_callback, 50)
        self.get_logger().info(
            f'Escorregamento pela inclinação: ganho {self.gain:.3f}, '
            f'offset do roll {p("roll_offset_deg").value:.2f} graus.')

    def imu_callback(self, message):
        self.roll = roll_of(message.orientation)

    def odom_callback(self, message):
        if self.roll is not None:
            vy = slope_lateral_velocity(
                self.gain, self.roll, self.offset,
                message.twist.twist.linear.x)
            twist = message.twist
            twist.twist.linear.y += vy
            std = math.sqrt(max(twist.covariance[7], 0.0))
            twist.covariance[7] = (std + self.uncertainty * abs(vy)) ** 2
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = SlopeSlip()
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
