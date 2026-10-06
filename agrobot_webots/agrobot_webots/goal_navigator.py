#!/usr/bin/env python3
"""Leva o Agrobot até um objetivo marcado no RViz ("2D Goal Pose").

Sem mapa e sem Nav2. A posição vem só da odometria (/odom). O LiDAR
frontal (/scan) serve para ver se a faixa da largura do robô, na direção
do objetivo, está livre. Se estiver, o robô segue direto. Se não, ele
escolhe a direção livre mais próxima da do objetivo, anda por ela até o
caminho para o objetivo abrir de novo e então volta a apontar para ele.

O LiDAR do Webots fica na frente do robô e só vê ~170° à frente, então
não enxerga o que está ao lado da carroceria. Para não bater a lateral ou a
traseira ao girar, o nó guarda os pontos que o LiDAR viu perto do robô
(memória curta: some quando o robô se afasta, não é um mapa) e, antes de
cada giro, confere se a carroceria vai encostar em algum deles.

Tópicos:
    /goal_pose  (geometry_msgs/PoseStamped)  entrada, publicado pelo RViz
    /odom       (nav_msgs/Odometry)          entrada
    /scan       (sensor_msgs/LaserScan)      entrada
    /cmd_vel    (geometry_msgs/Twist)        saída
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

IDLE = 'IDLE'
GO_TO_GOAL = 'GO_TO_GOAL'
ALIGN = 'ALIGN'

MEMORY_CELL = 0.05      # m, resolução da memória curta de pontos
ROTATION_LOOKAHEAD = 0.25   # rad, giro simulado antes de girar de fato
ESCAPE_STEP = 0.10      # m, deslocamento simulado antes de recuar/avançar
MOTION_LOOKAHEAD = 0.5  # s, movimento simulado ao andar em direção ao objetivo
DIRECTION_STEP = math.radians(5.0)  # passo da busca por direção livre
SIDE_SWITCH_PENALTY = 0.5   # rad, custo de trocar o lado do desvio


def wrap_angle(angle):
    """Normaliza um ângulo para [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def scan_points(scan):
    """Converte o LaserScan em pontos (x, y) no frame do LiDAR."""
    points = []
    for i, r in enumerate(scan.ranges):
        if math.isnan(r) or math.isinf(r):
            continue
        if r < scan.range_min or r > scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        points.append((r * math.cos(angle), r * math.sin(angle)))
    return points


def corridor_distance(points, half_width):
    """Distância até o obstáculo mais próximo no corredor à frente.

    O corredor é a faixa |y| < half_width, com x > 0. Retorna inf se
    estiver livre.
    """
    nearest = math.inf
    for x, y in points:
        if x > 0.0 and abs(y) < half_width:
            nearest = min(nearest, x)
    return nearest


def side_clearance(points, left, max_range):
    """Menor distância a um ponto no setor lateral (20° a 90°)."""
    nearest = max_range
    for x, y in points:
        angle = math.atan2(y, x)
        if not left:
            angle = -angle
        if math.radians(20.0) <= angle <= math.radians(90.0):
            nearest = min(nearest, math.hypot(x, y))
    return nearest


class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.goal_frame = param('goal_frame', 'odom')
        self.max_linear = param('max_linear_speed', 0.3)
        self.max_angular = param('max_angular_speed', 0.8)
        self.k_linear = param('k_linear', 0.5)
        self.k_angular = param('k_angular', 1.5)
        # Erro de direção acima do qual o robô gira parado antes de andar.
        self.turn_in_place = param('turn_in_place_angle', 0.6)
        self.goal_tolerance = param('goal_tolerance', 0.25)
        self.yaw_tolerance = param('yaw_tolerance', 0.15)
        # Carroceria (rodas incluídas), em relação ao base_link.
        self.robot_half_length = param('robot_half_length', 0.60)
        self.robot_half_width = param('robot_half_width', 0.56)
        self.safety_margin = param('safety_margin', 0.10)
        # LiDAR à frente do base_link (front_lidar_joint no URDF).
        self.lidar_x_offset = param('lidar_x_offset', 0.65)
        # Espaço livre exigido à frente da carroceria: para seguir direto
        # ao objetivo (stop) e para escolher uma direção de desvio (clear).
        self.stop_distance = param('stop_distance', 0.6)
        self.clear_distance = param('clear_distance', 1.0)
        self.avoid_angular = param('avoid_angular_speed', 0.5)
        self.escape_speed = param('escape_speed', 0.15)
        # Desiste se não chegar 0,1 m mais perto do objetivo nesse tempo.
        self.progress_timeout = param('progress_timeout', 40.0)
        # Quanto o robô pode andar para liberar giros sem progredir.
        self.max_escape_distance = param('max_escape_distance', 0.8)
        # Memória curta dos pontos vistos perto do robô.
        self.memory_time = param('memory_time', 300.0)
        self.memory_radius = param('memory_radius', 2.0)

        self.state = IDLE
        self.goal = None          # (x, y, yaw) no frame odom
        self.pose = None          # (x, y, yaw) no frame odom
        self.scan = None          # pontos do último scan, frame do LiDAR
        self.scan_half_fov = math.pi
        self.scan_max_range = 8.0
        self.memory = {}          # célula -> (x, y, t) no frame odom
        self.points = []          # scan + memória, frame do LiDAR
        self.avoid_direction = 0  # +1 esquerda, -1 direita, 0 sem desvio
        self.detour_heading = None  # rumo de desvio escolhido (frame odom)
        self.steps = 0            # passos de controle (10 Hz) neste objetivo
        self.best_distance = math.inf
        self.last_progress_step = 0
        self.escape_direction = 0
        self.escape_travel = 0.0
        self.blocked_steps = 0
        self.stopped = True

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        # O laço de controle usa o relógio do sistema. Com use_sim_time, um
        # timer no relógio do ROS só anda se alguém publicar /clock, e a
        # simulação do Webots daqui não publica (não usa o Ros2Supervisor):
        # o timer nunca dispararia e o robô não andaria.
        self.create_timer(0.1, self.control_loop,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            'Aguardando objetivo do RViz (2D Goal Pose em /goal_pose, '
            f'frame "{self.goal_frame}").'
        )

    # ------------------------------------------------------------------
    # Callbacks

    def goal_callback(self, msg):
        frame = msg.header.frame_id.lstrip('/')
        if frame and frame != self.goal_frame:
            self.get_logger().error(
                f'Objetivo no frame "{frame}" ignorado: este nó só usa '
                f'"{self.goal_frame}". No RViz, use Fixed Frame = '
                f'{self.goal_frame}.'
            )
            return

        p = msg.pose.position
        self.set_goal(p.x, p.y, yaw_from_quaternion(msg.pose.orientation))

    def set_goal(self, x, y, yaw):
        self.goal = (x, y, yaw)
        self.avoid_direction = 0
        self.detour_heading = None
        self.steps = 0
        self.best_distance = math.inf
        self.last_progress_step = 0
        self.escape_direction = 0
        self.escape_travel = 0.0
        self.blocked_steps = 0
        self.state = GO_TO_GOAL
        self.get_logger().info(
            f'Novo objetivo: x={x:.2f} y={y:.2f} '
            f'yaw={math.degrees(yaw):.0f}°'
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def scan_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        half_fov = max(abs(msg.angle_min), abs(msg.angle_max))
        self.add_scan(scan_points(msg), half_fov, msg.range_max, stamp)

    def add_scan(self, points, half_fov, max_range, stamp):
        """Guarda o scan e acrescenta à memória os pontos perto do robô."""
        self.scan = points
        self.scan_half_fov = half_fov
        self.scan_max_range = max_range
        if self.pose is None:
            return

        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        lx0 = x + self.lidar_x_offset * c
        ly0 = y + self.lidar_x_offset * s
        for px, py in points:
            if math.hypot(px, py) > self.memory_radius:
                continue
            wx = lx0 + px * c - py * s
            wy = ly0 + px * s + py * c
            key = (round(wx / MEMORY_CELL), round(wy / MEMORY_CELL))
            self.memory[key] = (wx, wy, stamp)

        for key, (wx, wy, t) in list(self.memory.items()):
            if stamp - t > self.memory_time or \
                    math.hypot(wx - x, wy - y) > self.memory_radius + 1.0:
                del self.memory[key]

    def local_points(self):
        """Scan atual + pontos da memória fora do campo de visão do LiDAR.

        Dentro do campo de visão vale só o scan atual, para um obstáculo
        que saiu do lugar (ou um desvio da odometria) não ficar preso na
        memória.
        """
        points = list(self.scan)
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        lx0 = x + self.lidar_x_offset * c
        ly0 = y + self.lidar_x_offset * s
        visible = self.scan_half_fov - math.radians(5.0)
        for wx, wy, _ in self.memory.values():
            dx, dy = wx - lx0, wy - ly0
            px = dx * c + dy * s
            py = -dx * s + dy * c
            if px > 0.0 and abs(math.atan2(py, px)) < visible:
                continue
            points.append((px, py))
        return points

    # ------------------------------------------------------------------
    # Controle

    def control_loop(self):
        if self.state == IDLE:
            return

        if self.pose is None or self.scan is None:
            self.get_logger().warning(
                'Esperando /odom e /scan...', throttle_duration_sec=2.0)
            return

        linear, angular = self.compute_command()
        self.publish(linear, angular)

    def compute_command(self):
        """Executa um passo de controle e retorna (v, w)."""
        self.points = self.local_points()
        x, y, yaw = self.pose
        gx, gy, gyaw = self.goal
        distance = math.hypot(gx - x, gy - y)
        goal_bearing = wrap_angle(math.atan2(gy - y, gx - x) - yaw)

        if self.state == GO_TO_GOAL:
            if distance < self.goal_tolerance:
                self.state = ALIGN
                self.get_logger().info(
                    'Posição alcançada. Ajustando orientação.')
                return 0.0, 0.0

            # Desiste se não chega mais perto do objetivo há muito tempo.
            self.steps += 1
            if distance < self.best_distance - 0.1:
                self.best_distance = distance
                self.last_progress_step = self.steps
                self.escape_travel = 0.0
            elif self.steps - self.last_progress_step > \
                    self.progress_timeout * 10:
                self.get_logger().error(
                    f'Sem chegar mais perto do objetivo há '
                    f'{self.progress_timeout:.0f} s. Parando; marque outro '
                    'objetivo.')
                self.state = IDLE
                return 0.0, 0.0

            direction = self.choose_direction(goal_bearing, distance)
            if direction is None:
                # Nenhuma direção livre à frente: gira para o lado mais
                # livre até abrir uma.
                if self.avoid_direction == 0:
                    self.avoid_direction = self.freer_side(goal_bearing)
                    side = 'esquerda' if self.avoid_direction > 0 \
                        else 'direita'
                    self.get_logger().info(
                        f'Caminho bloqueado à frente. Girando para a {side}.')
                return self.rotate(self.avoid_direction * self.avoid_angular)

            if direction != goal_bearing and self.avoid_direction == 0:
                self.avoid_direction = 1 if direction > goal_bearing else -1
                side = 'esquerda' if self.avoid_direction > 0 else 'direita'
                self.get_logger().info(
                    f'Obstáculo no caminho. Desviando pela {side}.')
            elif direction == goal_bearing and self.avoid_direction != 0:
                self.avoid_direction = 0
                self.get_logger().info('Caminho livre até o objetivo.')

            angular = self.clamp(self.k_angular * direction, self.max_angular)
            if abs(direction) > self.turn_in_place:
                return self.rotate(angular)

            linear = min(self.max_linear, self.k_linear * distance)
            linear *= math.cos(direction)
            # Andando e virando ao mesmo tempo, a lateral pode encostar em
            # algo que já saiu do campo de visão. Se encostaria, segue reto.
            if self.footprint_hits(linear * MOTION_LOOKAHEAD,
                                   angular * MOTION_LOOKAHEAD):
                angular = 0.0
            return linear, angular

        if self.state == ALIGN:
            yaw_error = wrap_angle(gyaw - yaw)
            if abs(yaw_error) < self.yaw_tolerance:
                self.state = IDLE
                self.get_logger().info('Objetivo alcançado.')
                return 0.0, 0.0
            angular = self.clamp(self.k_angular * yaw_error, self.max_angular)
            if self.footprint_hits(0.0, math.copysign(ROTATION_LOOKAHEAD,
                                                      angular)):
                # Sair do lugar para girar afastaria o robô do objetivo.
                self.state = IDLE
                self.get_logger().warning(
                    'Objetivo alcançado, mas sem espaço para girar até a '
                    'orientação pedida.')
                return 0.0, 0.0
            return 0.0, angular

        return 0.0, 0.0

    def free_distance(self, bearing):
        """Espaço livre à frente do robô se ele andasse na direção bearing.

        Considera a faixa da largura do robô (mais margem) que sai do
        centro do robô nessa direção, e desconta o meio comprimento dele.
        """
        half_width = self.robot_half_width + self.safety_margin
        c, s = math.cos(bearing), math.sin(bearing)
        nearest = math.inf
        for lx, ly in self.points:
            bx = lx + self.lidar_x_offset   # ponto no frame base_link
            along = bx * c + ly * s
            if along <= 0.0:
                continue
            if abs(-bx * s + ly * c) < half_width:
                nearest = min(nearest, along)
        return nearest - self.robot_half_length

    def choose_direction(self, goal_bearing, distance):
        """Direção (relativa ao robô) para andar, ou None se não houver.

        Vai direto ao objetivo se essa direção está livre até ele (ou por
        stop_distance). Senão escolhe, dentro do campo de visão, a direção
        livre mais próxima da do objetivo, preferindo continuar desviando
        pelo mesmo lado.
        """
        need = min(self.stop_distance,
                   max(distance - self.robot_half_length, 0.0))
        if self.free_distance(goal_bearing) >= need:
            self.detour_heading = None
            return goal_bearing

        # Mantém a direção de desvio já escolhida (em relação ao mundo)
        # enquanto ela continuar livre, para não ficar girando de um lado
        # para o outro.
        yaw = self.pose[2]
        if self.detour_heading is not None:
            bearing = wrap_angle(self.detour_heading - yaw)
            if self.free_distance(bearing) >= self.stop_distance:
                return bearing
            self.detour_heading = None

        best, best_cost = None, math.inf
        limit = self.scan_half_fov - math.radians(5.0)
        steps = int(limit / DIRECTION_STEP)
        for i in range(-steps, steps + 1):
            bearing = i * DIRECTION_STEP
            if self.free_distance(bearing) < self.clear_distance:
                continue
            cost = abs(wrap_angle(bearing - goal_bearing))
            if self.avoid_direction and \
                    (bearing - goal_bearing) * self.avoid_direction < 0.0:
                cost += SIDE_SWITCH_PENALTY
            if cost < best_cost:
                best, best_cost = bearing, cost
        if best is not None:
            self.detour_heading = wrap_angle(yaw + best)
        return best

    def freer_side(self, goal_bearing):
        """+1 (esquerda) ou -1 (direita): o lado mais livre para girar."""
        left = side_clearance(self.points, True, self.scan_max_range)
        right = side_clearance(self.points, False, self.scan_max_range)
        # Pequena preferência pelo lado em que está o objetivo.
        if goal_bearing > 0.0:
            left += 0.5
        else:
            right += 0.5
        # Um lado em que a carroceria nem consegue girar não serve.
        if self.footprint_hits(0.0, ROTATION_LOOKAHEAD):
            left = -math.inf
        if self.footprint_hits(0.0, -ROTATION_LOOKAHEAD):
            right = -math.inf
        return 1 if left >= right else -1

    def footprint_hits(self, dx, dtheta):
        """True se, após andar dx e girar dtheta, a carroceria encostar.

        Só contam pontos que ainda não estão encostando, senão um ponto já
        colado no robô bloquearia qualquer movimento.
        """
        half_length = self.robot_half_length + self.safety_margin / 2.0
        half_width = self.robot_half_width + self.safety_margin / 2.0
        c, s = math.cos(dtheta), math.sin(dtheta)
        for lx, ly in self.points:
            bx = lx + self.lidar_x_offset   # ponto no frame base_link
            if abs(bx) > 1.5 or abs(ly) > 1.5:
                continue
            if abs(bx) < half_length and abs(ly) < half_width:
                continue
            tx = bx - dx
            nx = tx * c + ly * s
            ny = -tx * s + ly * c
            if abs(nx) < half_length and abs(ny) < half_width:
                return True
        return False

    def rotate(self, angular):
        """Gira parado, desde que a carroceria não vá encostar em nada.

        Se o giro bater, tenta, nesta ordem: avançar um pouco, recuar um
        pouco (no máximo max_escape_distance), girar para o outro lado.
        Mantém o sentido de fuga até o giro ficar livre, para não ficar
        indo e voltando.
        """
        turn = math.copysign(ROTATION_LOOKAHEAD, angular)
        if not self.footprint_hits(0.0, turn):
            self.escape_direction = 0
            self.blocked_steps = 0
            return 0.0, angular

        options = [self.escape_direction] if self.escape_direction else []
        options += [1, -1]
        front = corridor_distance(
            self.points, self.robot_half_width + self.safety_margin)
        for direction in options:
            if self.escape_travel >= self.max_escape_distance:
                break
            # Avançar não pode levar o robô para cima do que está à frente.
            if direction > 0 and front < self.stop_distance / 2.0:
                continue
            if not self.footprint_hits(direction * ESCAPE_STEP, 0.0):
                self.escape_direction = direction
                self.escape_travel += self.escape_speed * 0.1
                return direction * self.escape_speed, 0.0

        self.escape_direction = 0
        if not self.footprint_hits(0.0, -turn):
            return 0.0, -angular

        self.blocked_steps += 1
        if self.blocked_steps > 50:
            self.get_logger().error(
                'Robô cercado: não consegue girar nem sair do lugar. Parando.')
            self.state = IDLE
        return 0.0, 0.0

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def publish(self, linear, angular):
        stopped = linear == 0.0 and angular == 0.0
        # Publica parado só uma vez, para não brigar com teleop quando IDLE.
        if stopped and self.stopped:
            return
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)
        self.stopped = stopped


def main(args=None):
    # Sem os tratadores de sinal do rclpy, o Ctrl+C chega como
    # KeyboardInterrupt com o contexto ainda válido, e dá para mandar o robô
    # parar antes de desligar. Com eles (o padrão), o contexto já estaria
    # fechado e o publish do finally falharia.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
