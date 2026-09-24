#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class StabilityMonitor(Node):

    def __init__(self):
        super().__init__("stability_monitor")

        # Limites provisórios em graus.
        self.roll_warning = 10.0
        self.roll_critical = 15.0

        self.pitch_warning = 12.0
        self.pitch_critical = 18.0

        # Histerese para impedir o estado de oscilar rapidamente.
        self.hysteresis = 2.0

        # Número de leituras críticas consecutivas necessárias.
        self.confirmation_samples = 5
        self.danger_counter = 0

        self.current_state = "NORMAL"
        self.current_direction = "SEM_RISCO"

        self.last_print_time = self.get_clock().now()
        self.last_publish_time = self.get_clock().now()

        # Intervalos das mensagens apresentadas.
        self.normal_interval = 2.0
        self.warning_interval = 1.0

        self.last_published_state = ""
        self.last_published_direction = ""

        self.imu_subscriber = self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.status_publisher = self.create_publisher(
            String,
            "/stability_status",
            10
        )

        self.warning_publisher = self.create_publisher(
            String,
            "/rollover_warning",
            10
        )

        self.get_logger().info(
            "Monitor de estabilidade iniciado."
        )

        self.get_logger().info(
            "A monitorizar capotamento lateral e longitudinal."
        )

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        # Roll: rotação em torno do eixo X.
        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sin_roll, cos_roll)

        # Pitch: rotação em torno do eixo Y.
        sin_pitch = 2.0 * (w * y - z * x)

        if abs(sin_pitch) >= 1.0:
            pitch = math.copysign(
                math.pi / 2.0,
                sin_pitch
            )
        else:
            pitch = math.asin(sin_pitch)

        # Yaw: direção do robô.
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        return (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw)
        )

    def determine_direction(self, roll, pitch):
        """
        Identifica qual inclinação domina.

        Os nomes esquerda/direita e frente/trás devem ser
        confirmados experimentalmente, pois dependem da orientação
        do imu_link no robô.
        """

        roll_ratio = abs(roll) / self.roll_warning
        pitch_ratio = abs(pitch) / self.pitch_warning

        if roll_ratio >= pitch_ratio:
            if roll > 0.0:
                return "DIREITA"

            return "ESQUERDA"

        if pitch > 0.0:
            return "TRAS"

        return "FRENTE"

    def raw_state(self, roll, pitch):
        abs_roll = abs(roll)
        abs_pitch = abs(pitch)

        if (
            abs_roll >= self.roll_critical
            or abs_pitch >= self.pitch_critical
        ):
            return "CRITICO"

        if (
            abs_roll >= self.roll_warning
            or abs_pitch >= self.pitch_warning
        ):
            return "AVISO"

        return "NORMAL"

    def apply_confirmation(self, requested_state):
        """
        Exige várias leituras perigosas consecutivas antes
        de declarar CRÍTICO.
        """

        if requested_state == "CRITICO":
            self.danger_counter += 1

            if self.danger_counter >= self.confirmation_samples:
                return "CRITICO"

            return "AVISO"

        self.danger_counter = 0
        return requested_state

    def apply_hysteresis(self, state, roll, pitch):
        """
        Evita que o estado fique alternando rapidamente
        junto dos limites.
        """

        abs_roll = abs(roll)
        abs_pitch = abs(pitch)

        if self.current_state == "CRITICO":
            remain_critical = (
                abs_roll >= self.roll_critical - self.hysteresis
                or abs_pitch >= self.pitch_critical - self.hysteresis
            )

            if remain_critical:
                return "CRITICO"

        if self.current_state == "AVISO":
            remain_warning = (
                abs_roll >= self.roll_warning - self.hysteresis
                or abs_pitch >= self.pitch_warning - self.hysteresis
            )

            if remain_warning and state == "NORMAL":
                return "AVISO"

        return state

    def publish_status(
        self,
        state,
        direction,
        roll,
        pitch,
        yaw
    ):
        status_message = String()

        status_message.data = (
            f"{state};"
            f"direcao={direction};"
            f"roll={roll:.2f};"
            f"pitch={pitch:.2f};"
            f"yaw={yaw:.2f}"
        )

        self.status_publisher.publish(status_message)

        warning_message = String()

        if state == "NORMAL":
            warning_message.data = "SEM RISCO DE CAPOTAMENTO"

        elif state == "AVISO":
            warning_message.data = (
                f"AVISO: risco de capotamento para {direction}"
            )

        else:
            warning_message.data = (
                f"PERIGO CRITICO: capotamento iminente "
                f"para {direction}"
            )

        self.warning_publisher.publish(warning_message)

    def print_status(
        self,
        state,
        direction,
        roll,
        pitch
    ):
        now = self.get_clock().now()

        elapsed = (
            now - self.last_print_time
        ).nanoseconds / 1e9

        if state == "NORMAL":
            interval = self.normal_interval
        else:
            interval = self.warning_interval

        state_changed = (
            state != self.current_state
            or direction != self.current_direction
        )

        if elapsed < interval and not state_changed:
            return

        if state == "NORMAL":
            self.get_logger().info(
                f"NORMAL | "
                f"Roll: {roll:.2f}° | "
                f"Pitch: {pitch:.2f}°"
            )

        elif state == "AVISO":
            self.get_logger().warning(
                f"AVISO DE CAPOTAMENTO PARA {direction} | "
                f"Roll: {roll:.2f}° | "
                f"Pitch: {pitch:.2f}°"
            )

        else:
            print("\a", end="", flush=True)

            self.get_logger().error(
                f"PERIGO CRÍTICO: CAPOTAMENTO PARA {direction} | "
                f"Roll: {roll:.2f}° | "
                f"Pitch: {pitch:.2f}°"
            )

        self.last_print_time = now


    def publish_status_throttled(
        self,
        state,
        direction,
        roll,
        pitch,
        yaw
    ):
        now = self.get_clock().now()

        elapsed = (
            now - self.last_publish_time
        ).nanoseconds / 1e9

        if state == "NORMAL":
            interval = self.normal_interval
        else:
            interval = self.warning_interval

        state_changed = (
            state != self.last_published_state
            or direction != self.last_published_direction
        )

        if elapsed < interval and not state_changed:
            return

        self.publish_status(
            state,
            direction,
            roll,
            pitch,
            yaw
        )

        self.last_publish_time = now
        self.last_published_state = state
        self.last_published_direction = direction

    def imu_callback(self, msg):
        q = msg.orientation

        roll, pitch, yaw = self.quaternion_to_euler(
            q.x,
            q.y,
            q.z,
            q.w
        )

        requested_state = self.raw_state(
            roll,
            pitch
        )

        confirmed_state = self.apply_confirmation(
            requested_state
        )

        state = self.apply_hysteresis(
            confirmed_state,
            roll,
            pitch
        )

        if state == "NORMAL":
            direction = "SEM_RISCO"
        else:
            direction = self.determine_direction(
                roll,
                pitch
            )

        self.publish_status_throttled(
            state,
            direction,
            roll,
            pitch,
            yaw
        )

        self.print_status(
            state,
            direction,
            roll,
            pitch
        )

        if (
            state != self.current_state
            or direction != self.current_direction
        ):
            self.get_logger().warning(
                f"Alteração: "
                f"{self.current_state}/{self.current_direction} "
                f"-> {state}/{direction}"
            )

            self.current_state = state
            self.current_direction = direction


def main(args=None):
    rclpy.init(args=args)

    node = StabilityMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()