#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Velocidades
        self.forward_speed = 0.20
        self.turn_speed = 0.45

        # Distâncias
        self.stop_distance = 1.20
        self.clear_distance = 1.55

        # Estados possíveis:
        # FORWARD, TURN_LEFT, TURN_RIGHT
        self.state = "FORWARD"

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            "Autonomia iniciada. À espera do LiDAR..."
        )

    @staticmethod
    def valid_range(value):
        if math.isnan(value) or math.isinf(value):
            return 12.0

        if value <= 0.0:
            return 12.0

        return value

    def scan_callback(self, msg):
        ranges = [
            self.valid_range(value)
            for value in msg.ranges
        ]

        if len(ranges) < 360:
            self.get_logger().warning(
                f"Leitura LiDAR incompleta: {len(ranges)} pontos"
            )
            return

        # Para o teu LiDAR:
        # 180 = frente
        # 270 = esquerda
        # 90 = direita

        front = min(ranges[160:201])
        front_left = min(ranges[201:250])
        left = min(ranges[250:310])

        front_right = min(ranges[110:159])
        right = min(ranges[50:110])

        cmd = Twist()

        # ESTADO: ANDAR EM FRENTE
        if self.state == "FORWARD":

            if front < self.stop_distance:

                cmd.linear.x = 0.0

                # Escolhe uma direção uma única vez
                left_space = min(front_left, left)
                right_space = min(front_right, right)

                if left_space >= right_space:
                    self.state = "TURN_LEFT"
                    self.get_logger().info(
                        f"Obstáculo a {front:.2f} m. "
                        "Iniciando curva para a esquerda."
                    )
                else:
                    self.state = "TURN_RIGHT"
                    self.get_logger().info(
                        f"Obstáculo a {front:.2f} m. "
                        "Iniciando curva para a direita."
                    )

            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0

        # ESTADO: VIRAR À ESQUERDA
        elif self.state == "TURN_LEFT":

            cmd.linear.x = 0.03
            cmd.angular.z = self.turn_speed

            # Só termina a curva quando houver margem suficiente
            if front > self.clear_distance and front_left > 1.10:
                self.state = "FORWARD"
                self.get_logger().info(
                    "Frente livre. Voltando a avançar."
                )

        # ESTADO: VIRAR À DIREITA
        elif self.state == "TURN_RIGHT":

            cmd.linear.x = 0.03
            cmd.angular.z = -self.turn_speed

            if front > self.clear_distance and front_right > 1.10:
                self.state = "FORWARD"
                self.get_logger().info(
                    "Frente livre. Voltando a avançar."
                )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()