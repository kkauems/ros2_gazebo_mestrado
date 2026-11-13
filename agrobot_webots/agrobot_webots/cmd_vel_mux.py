#!/usr/bin/env python3
"""
Junta o comando do operador (joystick ou teclado) e o do Nav2 num /cmd_vel só.

O operador tem prioridade: enquanto chegam mensagens em /cmd_vel_teleop, o
comando do Nav2 é ignorado. Quando o operador para de mandar por mais de
teleop_timeout (soltou o botão de habilitar do joystick ou parou o teclado),
o Nav2 volta a comandar, sem cancelar a missão. Assim o operador pode tomar o
controle no meio de uma missão, desviar o robô e devolvê-lo.

Uma entrada só vale enquanto for recente (timeout). Quando a entrada ativa
fica velha, o nó manda um zero e para de publicar; o driver também para o
robô se o /cmd_vel parar de chegar.

Tópicos:
    /cmd_vel_teleop      (geometry_msgs/Twist)  entrada, prioridade alta
    /cmd_vel_nav_out     (geometry_msgs/Twist)  entrada, saída do Nav2
    /cmd_vel             (geometry_msgs/Twist)  saída, para o driver
    /cmd_vel_mux/source  (std_msgs/String)      quem comanda: teleop, nav
                                                ou none (só quando muda)
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

TELEOP = 'teleop'
NAV = 'nav'
NONE = 'none'


class MuxState:
    """
    Lógica do mux, sem ROS (para testar).

    Os tempos são em segundos, de qualquer relógio, desde que o mesmo.
    """

    def __init__(self, teleop_timeout, nav_timeout):
        self.timeouts = {TELEOP: teleop_timeout, NAV: nav_timeout}
        self.last = {TELEOP: None, NAV: None}
        self.commands = {TELEOP: None, NAV: None}

    def receive(self, source, command, now):
        self.last[source] = now
        self.commands[source] = command

    def fresh(self, source, now):
        last = self.last[source]
        return last is not None and now - last <= self.timeouts[source]

    def active(self, now):
        """Fonte que comanda agora: teleop, nav ou none."""
        for source in (TELEOP, NAV):
            if self.fresh(source, now):
                return source
        return NONE

    def command(self, now):
        """Comando a publicar agora, ou None se nenhuma entrada é recente."""
        source = self.active(now)
        if source == NONE:
            return None
        return self.commands[source]


class CmdVelMux(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('teleop_timeout', 0.5)
        self.declare_parameter('nav_timeout', 0.5)
        self.declare_parameter('rate', 20.0)
        self.state = MuxState(
            self.get_parameter('teleop_timeout').value,
            self.get_parameter('nav_timeout').value)
        self.source = NONE

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_publisher = self.create_publisher(
            String, '/cmd_vel_mux/source', latched)
        self.source_publisher.publish(String(data=self.source))
        self.create_subscription(
            Twist, '/cmd_vel_teleop',
            lambda message: self.receive(TELEOP, message), 10)
        self.create_subscription(
            Twist, '/cmd_vel_nav_out',
            lambda message: self.receive(NAV, message), 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive(self, source, message):
        self.state.receive(source, message, self.now())
        # Publica já: o atraso do timer somaria ao do Nav2.
        if self.state.active(self.now()) == source:
            self.publisher.publish(message)
            self.set_source(source)

    def tick(self):
        now = self.now()
        source = self.state.active(now)
        if source == NONE and self.source != NONE:
            # A entrada que comandava ficou velha: para o robô uma vez.
            self.publisher.publish(Twist())
        self.set_source(source)

    def set_source(self, source):
        if source == self.source:
            return
        self.get_logger().info(f'Comando: {self.source} -> {source}')
        self.source = source
        self.source_publisher.publish(String(data=source))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
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
