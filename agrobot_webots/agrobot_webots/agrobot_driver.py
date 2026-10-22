import array
import ctypes
import math
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster

# Variâncias da /imu/data. O IMU do Webots não tem ruído; estes valores só
# dizem ao EKF quanto confiar em cada medida.
IMU_ORIENTATION_VARIANCE = 0.0004   # (0.02 rad)^2, ~1 grau
IMU_GYRO_VARIANCE = 0.0001          # (0.01 rad/s)^2
IMU_ACCEL_VARIANCE = 0.01           # (0.1 m/s^2)^2

# Desvio-padrão das velocidades das rodas em /wheel/odom: um piso pequeno
# mais uma parte proporcional ao movimento (como o modelo de odometria do
# Probabilistic Robotics). Parado, o EKF quase não acumula incerteza; andando,
# ela cresce mais ao longo do movimento (vx) do que de lado (vy).
WHEEL_STD_FLOOR = 0.002             # m/s
WHEEL_VX_STD_PER_SPEED = 0.10       # 10% de |v|
WHEEL_VY_STD_PER_SPEED = 0.05       # deriva lateral: 5% de |v| ...
WHEEL_VY_STD_PER_TURN = 0.05        # ... + 0,05 m/s por rad/s (skid-steer)

# Câmera RGB-D do VSLAM (VslamCamera.proto). Nomes de tópico como os do
# realsense-ros; a profundidade já sai registrada à cor.
VSLAM_COLOR_DEVICE = 'vslam_color'
VSLAM_DEPTH_DEVICE = 'vslam_depth'
VSLAM_FRAME = 'vslam_camera_optical_frame'
VSLAM_COLOR_TOPIC = '/vslam_camera/color/image_raw'
VSLAM_COLOR_INFO_TOPIC = '/vslam_camera/color/camera_info'
VSLAM_DEPTH_TOPIC = '/vslam_camera/aligned_depth_to_color/image_raw'
VSLAM_DEPTH_INFO_TOPIC = '/vslam_camera/aligned_depth_to_color/camera_info'
# Média do custo de publicação no log a cada tantos segundos de simulação.
VSLAM_LOG_PERIOD = 60.0

WHEEL_JOINTS = [
    'front_left_wheel_joint',
    'rear_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_right_wheel_joint',
]


class AgrobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.wheel_radius = float(properties.get('wheelRadius', 0.22))
        self.wheel_separation = float(properties.get('wheelSeparation', 0.96))
        # Skid-steer escorrega nas curvas: a separação efetiva é maior.
        self.wheel_separation_scale = float(
            properties.get('wheelSeparationScale', 1.0))
        # false quando o robot_localization (EKF) publica /odom e o TF
        # odom -> base_link; o simulation.launch.py troca com ekf:=true.
        self.publish_odom = properties.get(
            'publishOdom', 'true').strip().lower() == 'true'
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.left_motors = [
            self.get_motor('front_left_wheel_joint'),
            self.get_motor('rear_left_wheel_joint'),
        ]
        self.right_motors = [
            self.get_motor('front_right_wheel_joint'),
            self.get_motor('rear_right_wheel_joint'),
        ]
        self.left_sensors = [
            self.get_sensor('front_left_wheel_joint_sensor'),
            self.get_sensor('rear_left_wheel_joint_sensor'),
        ]
        self.right_sensors = [
            self.get_sensor('front_right_wheel_joint_sensor'),
            self.get_sensor('rear_right_wheel_joint_sensor'),
        ]
        # O yaw das rodas erra muito nos giros (skid-steer derrapa);
        # quando houver IMU, o yaw da odometria vem dela.
        self.imu = self.robot.getDevice('imu')
        if self.imu is not None:
            self.imu.enable(self.time_step)
        self.imu_yaw_offset = None
        self.gyro = self.robot.getDevice('gyro')
        if self.gyro is not None:
            self.gyro.enable(self.time_step)
        self.accelerometer = self.robot.getDevice('accelerometer')
        if self.accelerometer is not None:
            self.accelerometer.enable(self.time_step)

        # O Ros2Lidar do webots_ros2_driver carimba o /scan com tempo 0
        # (o relógio do nó dele não avança), e o Nav2 descarta tudo.
        # Por isso o scan é publicado aqui, com o mesmo relógio da odometria.
        self.lidar = self.robot.getDevice('front_lidar')
        self.lidar_period = 3 * self.time_step
        self.lidar.enable(self.lidar_period)
        # Ler a range image antes da primeira amostra derruba o controlador.
        self.next_scan_time = 2 * self.lidar_period / 1000.0

        self.init_vslam_camera(properties)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # Odometria só das rodas (yaw também das rodas), entrada do EKF.
        self.wheel_x = 0.0
        self.wheel_y = 0.0
        self.wheel_yaw = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = None

        rclpy.init(args=None)
        self.node = rclpy.create_node('agrobot_driver')
        self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        # Sem /clock, os nós com use_sim_time ficam com o tempo parado em 0
        # (timers não disparam, e os publicadores do webots_ros2_driver,
        # como câmera e range finder, carimbam tudo com 0).
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 10)
        if self.publish_odom:
            self.odom_publisher = self.node.create_publisher(
                Odometry, '/odom', 10)
            self.tf_broadcaster = TransformBroadcaster(self.node)
        self.wheel_odom_publisher = self.node.create_publisher(
            Odometry, '/wheel/odom', 10)
        self.imu_publisher = self.node.create_publisher(
            Imu, '/imu/data', qos_profile_sensor_data)
        self.scan_publisher = self.node.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.joint_state_publisher = self.node.create_publisher(
            JointState, '/joint_states', 10)
        if self.vslam_period_ms > 0:
            # Reliable: é o padrão do rgbd_sync do RTAB-Map; best effort,
            # como no /scan, não casaria com ele.
            camera_qos = QoSProfile(
                depth=5, reliability=ReliabilityPolicy.RELIABLE)
            self.vslam_color_publisher = self.node.create_publisher(
                Image, VSLAM_COLOR_TOPIC, camera_qos)
            self.vslam_color_info_publisher = self.node.create_publisher(
                CameraInfo, VSLAM_COLOR_INFO_TOPIC, camera_qos)
            self.vslam_depth_publisher = self.node.create_publisher(
                Image, VSLAM_DEPTH_TOPIC, camera_qos)
            self.vslam_depth_info_publisher = self.node.create_publisher(
                CameraInfo, VSLAM_DEPTH_INFO_TOPIC, camera_qos)
            self.node.get_logger().info(
                f'Câmera do VSLAM: {self.vslam_width} x {self.vslam_height} '
                f'a cada {self.vslam_period_ms} ms.')
        elif self.vslam_missing:
            self.node.get_logger().warn(
                f'{VSLAM_COLOR_DEVICE} ou {VSLAM_DEPTH_DEVICE} não existe no '
                'robô; a câmera do VSLAM fica desligada.')
        self.node.get_logger().info('Controlador Webots do agrobot iniciado.')

    def init_vslam_camera(self, properties):
        # O Ros2Camera e o Ros2RangeFinder do webots_ros2_driver carimbam com
        # o relógio do nó deles e rodam antes do /clock deste passo; cor e
        # profundidade são publicadas aqui, como o /scan, com um só carimbo.
        period = int(properties.get('vslamCameraPeriod', 96))
        self.vslam_period_ms = 0
        self.vslam_missing = False
        if period <= 0:
            return
        self.vslam_color = self.robot.getDevice(VSLAM_COLOR_DEVICE)
        self.vslam_depth = self.robot.getDevice(VSLAM_DEPTH_DEVICE)
        if self.vslam_color is None or self.vslam_depth is None:
            self.vslam_missing = True
            return
        if period % self.time_step != 0:
            raise ValueError(
                f'vslamCameraPeriod ({period} ms) tem que ser múltiplo do '
                f'passo da simulação ({self.time_step} ms).')
        width = self.vslam_color.getWidth()
        height = self.vslam_color.getHeight()
        fov = self.vslam_color.getFov()
        # Com resolução e FOV iguais, a profundidade fica registrada à cor.
        if (self.vslam_depth.getWidth(), self.vslam_depth.getHeight()) != (
                width, height) or abs(self.vslam_depth.getFov() - fov) > 1e-6:
            raise ValueError(
                'vslam_color e vslam_depth precisam ter a mesma resolução e '
                'o mesmo FOV.')
        self.vslam_period_ms = period
        self.vslam_width = width
        self.vslam_height = height
        self.vslam_camera_info = self.make_camera_info(width, height, fov)
        self.vslam_color.enable(period)
        self.vslam_depth.enable(period)
        # Os dois sensores atualizam a cada período contado de agora. Publicar
        # pelo tempo inteiro em ms (e não por "now >= próximo", que acumula
        # erro de ponto flutuante) garante que a imagem é a do carimbo.
        self.vslam_enable_ms = round(self.robot.getTime() * 1000.0)
        self.vslam_frames = 0
        self.vslam_cost = 0.0
        self.vslam_next_log = VSLAM_LOG_PERIOD

    @staticmethod
    def make_camera_info(width, height, fov):
        # Câmera pinhole sem distorção. O FOV do Webots é o horizontal e os
        # pixels são quadrados. Centro óptico na convenção do OpenCV (centro
        # do pixel 0 na coordenada 0): (w - 1) / 2.
        focal = width / (2.0 * math.tan(fov / 2.0))
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        info = CameraInfo()
        info.header.frame_id = VSLAM_FRAME
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def get_motor(self, name):
        motor = self.robot.getDevice(name)
        motor.setPosition(math.inf)
        motor.setVelocity(0.0)
        return motor

    def get_sensor(self, name):
        sensor = self.robot.getDevice(name)
        sensor.enable(self.time_step)
        return sensor

    def cmd_vel_callback(self, message):
        self.linear_velocity = message.linear.x
        self.angular_velocity = message.angular.z

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

        separation = self.wheel_separation * self.wheel_separation_scale
        left_speed = (
            self.linear_velocity
            - self.angular_velocity * separation / 2.0
        ) / self.wheel_radius
        right_speed = (
            self.linear_velocity
            + self.angular_velocity * separation / 2.0
        ) / self.wheel_radius

        for motor in self.left_motors:
            motor.setVelocity(left_speed)

        for motor in self.right_motors:
            motor.setVelocity(right_speed)

        now = self.robot.getTime()
        stamp = Time()
        stamp.sec = int(now)
        stamp.nanosec = int((now - int(now)) * 1e9)
        self.clock_publisher.publish(Clock(clock=stamp))
        self.update_odometry(separation, now, stamp)
        self.publish_imu(stamp)
        self.publish_joint_states(stamp)
        if now >= self.next_scan_time:
            self.next_scan_time = now + self.lidar_period / 1000.0
            self.publish_scan(stamp)
        if self.vslam_period_ms > 0:
            elapsed = round(now * 1000.0) - self.vslam_enable_ms
            # Como no LiDAR, só depois de 2 períodos: antes da primeira
            # amostra a imagem ainda não existe.
            if (elapsed >= 2 * self.vslam_period_ms
                    and elapsed % self.vslam_period_ms == 0):
                self.publish_vslam_camera(now, stamp)

    def publish_vslam_camera(self, now, stamp):
        start = time.perf_counter()
        try:
            color = self.vslam_color.getImage()
        except ValueError:
            # Ponteiro nulo: o Webots ainda não entregou imagem.
            return
        depth_pointer = self.vslam_depth.getRangeImage(data_type='buffer')
        if not color or not depth_pointer:
            return
        width = self.vslam_width
        height = self.vslam_height
        # array('B') copia os bytes de uma vez; por lista custaria vários ms.
        color = array.array('B', color)
        depth = array.array(
            'B', ctypes.string_at(depth_pointer, 4 * width * height))

        color_message = Image()
        color_message.header.stamp = stamp
        color_message.header.frame_id = VSLAM_FRAME
        color_message.width = width
        color_message.height = height
        color_message.encoding = 'bgra8'
        color_message.step = 4 * width
        color_message.data = color

        # Profundidade planar em metros (float32); fora de [minRange,
        # maxRange] o Webots devolve +inf, que o RTAB-Map trata como inválido.
        depth_message = Image()
        depth_message.header.stamp = stamp
        depth_message.header.frame_id = VSLAM_FRAME
        depth_message.width = width
        depth_message.height = height
        depth_message.encoding = '32FC1'
        depth_message.step = 4 * width
        depth_message.data = depth

        info = self.vslam_camera_info
        info.header.stamp = stamp
        self.vslam_color_publisher.publish(color_message)
        self.vslam_color_info_publisher.publish(info)
        self.vslam_depth_publisher.publish(depth_message)
        self.vslam_depth_info_publisher.publish(info)

        self.vslam_frames += 1
        self.vslam_cost += time.perf_counter() - start
        if now >= self.vslam_next_log:
            self.vslam_next_log = now + VSLAM_LOG_PERIOD
            self.node.get_logger().info(
                f'Câmera do VSLAM: {self.vslam_frames} quadros, '
                f'{1000.0 * self.vslam_cost / self.vslam_frames:.1f} ms '
                'por quadro para ler e publicar.')
            self.vslam_frames = 0
            self.vslam_cost = 0.0

    def publish_joint_states(self, stamp):
        sensors = [
            self.left_sensors[0],
            self.left_sensors[1],
            self.right_sensors[0],
            self.right_sensors[1],
        ]
        positions = [s.getValue() for s in sensors]
        if any(math.isnan(p) for p in positions):
            return
        message = JointState()
        message.header.stamp = stamp
        message.name = WHEEL_JOINTS
        message.position = positions
        self.joint_state_publisher.publish(message)

    def publish_scan(self, stamp):
        ranges = self.lidar.getRangeImage()
        if not ranges:
            return
        fov = self.lidar.getFov()
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = 'front_lidar_link'
        # O Webots entrega da esquerda (+fov/2) para a direita.
        message.angle_min = -fov / 2.0
        message.angle_max = fov / 2.0
        message.angle_increment = fov / (len(ranges) - 1)
        message.scan_time = self.lidar_period / 1000.0
        message.range_min = self.lidar.getMinRange()
        message.range_max = self.lidar.getMaxRange()
        message.ranges = [float(r) for r in reversed(ranges)]
        self.scan_publisher.publish(message)

    def publish_imu(self, stamp):
        if self.imu is None:
            return
        q = self.imu.getQuaternion()
        w = self.gyro.getValues() if self.gyro is not None else None
        a = (self.accelerometer.getValues()
             if self.accelerometer is not None else None)
        # Como os encoders, os sensores podem dar NaN antes da 1a amostra.
        if any(math.isnan(v) for v in [*q, *(w or []), *(a or [])]):
            return
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = 'imu_link'
        # O Webots devolve o quaternion como (x, y, z, w).
        message.orientation.x = q[0]
        message.orientation.y = q[1]
        message.orientation.z = q[2]
        message.orientation.w = q[3]
        message.orientation_covariance[0] = IMU_ORIENTATION_VARIANCE
        message.orientation_covariance[4] = IMU_ORIENTATION_VARIANCE
        message.orientation_covariance[8] = IMU_ORIENTATION_VARIANCE
        if w is not None:
            message.angular_velocity.x = w[0]
            message.angular_velocity.y = w[1]
            message.angular_velocity.z = w[2]
            for i in (0, 4, 8):
                message.angular_velocity_covariance[i] = IMU_GYRO_VARIANCE
        else:
            # -1 no primeiro elemento: "este campo não existe".
            message.angular_velocity_covariance[0] = -1.0
        if a is not None:
            message.linear_acceleration.x = a[0]
            message.linear_acceleration.y = a[1]
            message.linear_acceleration.z = a[2]
            for i in (0, 4, 8):
                message.linear_acceleration_covariance[i] = IMU_ACCEL_VARIANCE
        else:
            message.linear_acceleration_covariance[0] = -1.0
        self.imu_publisher.publish(message)

    def read_imu_yaw(self):
        if self.imu is None:
            return None
        yaw = self.imu.getRollPitchYaw()[2]
        if math.isnan(yaw):
            return None
        if self.imu_yaw_offset is None:
            self.imu_yaw_offset = yaw
        yaw -= self.imu_yaw_offset
        return math.atan2(math.sin(yaw), math.cos(yaw))

    def update_odometry(self, separation, now, stamp):
        left = sum(s.getValue() for s in self.left_sensors) / 2.0
        right = sum(s.getValue() for s in self.right_sensors) / 2.0

        # Os sensores retornam NaN no primeiro passo.
        if math.isnan(left) or math.isnan(right):
            return
        if self.last_left is None:
            self.last_left = left
            self.last_right = right
            self.last_time = now
            return

        d_left = (left - self.last_left) * self.wheel_radius
        d_right = (right - self.last_right) * self.wheel_radius
        dt = now - self.last_time
        self.last_left = left
        self.last_right = right
        self.last_time = now

        d_center = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / separation
        self.publish_wheel_odometry(d_center, d_yaw, dt, stamp)
        if not self.publish_odom:
            return
        imu_yaw = self.read_imu_yaw()
        if imu_yaw is not None:
            d_yaw = math.atan2(
                math.sin(imu_yaw - self.yaw), math.cos(imu_yaw - self.yaw))
        mid_yaw = self.yaw + d_yaw / 2.0
        self.x += d_center * math.cos(mid_yaw)
        self.y += d_center * math.sin(mid_yaw)
        self.yaw = math.atan2(
            math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        if dt > 0.0:
            odom.twist.twist.linear.x = d_center / dt
            odom.twist.twist.angular.z = d_yaw / dt
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def publish_wheel_odometry(self, d_center, d_yaw, dt, stamp):
        mid_yaw = self.wheel_yaw + d_yaw / 2.0
        self.wheel_x += d_center * math.cos(mid_yaw)
        self.wheel_y += d_center * math.sin(mid_yaw)
        self.wheel_yaw = math.atan2(
            math.sin(self.wheel_yaw + d_yaw), math.cos(self.wheel_yaw + d_yaw))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.wheel_x
        odom.pose.pose.position.y = self.wheel_y
        odom.pose.pose.orientation.z = math.sin(self.wheel_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.wheel_yaw / 2.0)
        v = d_center / dt if dt > 0.0 else 0.0
        w = d_yaw / dt if dt > 0.0 else 0.0
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # O EKF usa só vx e vy daqui. vy = 0 é a restrição de que o robô não
        # anda de lado; a variância diz quanto ele pode derrapar. O giro das
        # rodas erra muito (skid-steer derrapa), por isso a variância alta em
        # wz. A pose fica com variância fixa: o EKF não a usa.
        std_vx = WHEEL_STD_FLOOR + WHEEL_VX_STD_PER_SPEED * abs(v)
        std_vy = (WHEEL_STD_FLOOR + WHEEL_VY_STD_PER_SPEED * abs(v)
                  + WHEEL_VY_STD_PER_TURN * abs(w))
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.5
        odom.twist.covariance[0] = std_vx ** 2
        odom.twist.covariance[7] = std_vy ** 2
        odom.twist.covariance[35] = 0.5
        self.wheel_odom_publisher.publish(odom)
