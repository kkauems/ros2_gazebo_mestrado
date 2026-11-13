#!/usr/bin/env python3
"""
Teleoperação pelo teclado, para a simulação (no campo, use o joystick).

Rode num terminal próprio, com a simulação já aberta com teleop:=keyboard:

    ros2 run agrobot_webots teleop_keyboard

Funciona como um piloto automático: cada tecla muda a velocidade, e o robô
mantém a velocidade escolhida até a próxima tecla. Não é preciso segurar a
tecla (no WSL e em vários terminais, a repetição da tecla segurada tem um
atraso que faria o robô dar trancos).

Teclas:
    w / seta para cima     acelera para a frente (+0,1 m/s)
    s / seta para baixo    desacelera / ré (-0,1 m/s)
    a / seta à esquerda    gira mais para a esquerda (+0,2 rad/s)
    d / seta à direita     gira mais para a direita (-0,2 rad/s)
    espaço ou k            para e devolve o comando ao Nav2
    p                      segura o robô parado (o Nav2 não comanda) / solta
    q ou Ctrl-C            para e sai

Enquanto a velocidade não é zero, o nó publica em /cmd_vel_teleop a 10 Hz e
o cmd_vel_mux dá prioridade a ele sobre o Nav2. Parado (espaço), ele deixa de
publicar e, meio segundo depois, o Nav2 volta a comandar, se houver missão.
"""

import os
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

LINEAR_STEP = 0.1
ANGULAR_STEP = 0.2
MAX_LINEAR = 0.5    # igual ao máximo do Nav2
MAX_ANGULAR = 1.0

ARROWS = {'\x1b[A': 'w', '\x1b[B': 's', '\x1b[D': 'a', '\x1b[C': 'd'}

HELP = """
Teleoperação pelo teclado (publica em /cmd_vel_teleop)
  w/s ou setas cima/baixo   velocidade linear +/- 0,1 m/s
  a/d ou setas esq./dir.    giro +/- 0,2 rad/s
  espaço ou k               parar (o Nav2 volta a comandar)
  p                         segurar parado / soltar
  q                         sair
"""


def clamp(value, limit):
    return max(-limit, min(limit, value))


class KeyboardState:
    """Velocidade escolhida pelas teclas, sem ROS (para testar)."""

    def __init__(self):
        self.linear = 0.0
        self.angular = 0.0
        self.hold = False
        self.quit = False

    def press(self, key):
        key = ARROWS.get(key, key)
        if key == 'w':
            self.linear = clamp(self.linear + LINEAR_STEP, MAX_LINEAR)
        elif key == 's':
            self.linear = clamp(self.linear - LINEAR_STEP, MAX_LINEAR)
        elif key == 'a':
            self.angular = clamp(self.angular + ANGULAR_STEP, MAX_ANGULAR)
        elif key == 'd':
            self.angular = clamp(self.angular - ANGULAR_STEP, MAX_ANGULAR)
        elif key in (' ', 'k'):
            self.stop()
            self.hold = False
        elif key == 'p':
            self.stop()
            self.hold = not self.hold
        elif key in ('q', '\x03'):
            self.stop()
            self.hold = False
            self.quit = True
        else:
            return
        if self.moving():
            self.hold = False
        # Arredonda para não acumular 0,30000000000000004.
        self.linear = round(self.linear, 3)
        self.angular = round(self.angular, 3)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0

    def moving(self):
        return self.linear != 0.0 or self.angular != 0.0

    def publishing(self):
        """Publica enquanto anda ou enquanto segura o robô parado."""
        return self.moving() or self.hold

    def status(self):
        if self.hold:
            return 'segurando parado'
        return f'v = {self.linear:+.1f} m/s   w = {self.angular:+.1f} rad/s'


def read_key(timeout):
    """Uma tecla (ou a sequência de uma seta), ou '' se nada chegou."""
    # os.read, e não sys.stdin.read: o buffer do Python engoliria o resto da
    # sequência da seta e o select não a veria mais.
    fd = sys.stdin.fileno()
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return ''
    key = os.read(fd, 1)
    if key == b'\x1b':
        # Setas chegam como ESC [ A..D.
        ready, _, _ = select.select([fd], [], [], 0.01)
        if ready:
            key += os.read(fd, 2)
    return key.decode(errors='ignore')


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_teleop', 10)

    def publish(self, state):
        message = Twist()
        message.linear.x = state.linear
        message.angular.z = state.angular
        self.publisher.publish(message)


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop_keyboard precisa de um terminal: rode com ros2 run '
              'num terminal próprio.', file=sys.stderr)
        return 1
    # Sem o tratador de sinais do rclpy: o Ctrl-C vira KeyboardInterrupt e
    # o zero final ainda sai antes de desligar.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopKeyboard()
    state = KeyboardState()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last_status = None
        while rclpy.ok() and not state.quit:
            was_publishing = state.publishing()
            key = read_key(0.1)
            if key:
                state.press(key)
            if state.publishing():
                node.publish(state)
            elif was_publishing:
                # Um zero ao parar; depois silêncio, para o Nav2 voltar.
                node.publish(state)
            status = state.status()
            if status != last_status:
                print('\r' + status.ljust(50), end='', flush=True)
                last_status = status
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        state.stop()
    finally:
        node.publish(KeyboardState())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
