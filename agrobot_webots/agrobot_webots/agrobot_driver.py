import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster

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

        # O Ros2Lidar do webots_ros2_driver carimba o /scan com tempo 0
        # (o relógio do nó dele não avança), e o Nav2 descarta tudo.
        # Por isso o scan é publicado aqui, com o mesmo relógio da odometria.
        self.lidar = self.robot.getDevice('front_lidar')
        self.lidar_period = 3 * self.time_step
        self.lidar.enable(self.lidar_period)
        # Ler a range image antes da primeira amostra derruba o controlador.
        self.next_scan_time = 2 * self.lidar_period / 1000.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
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
        self.odom_publisher = self.node.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.scan_publisher = self.node.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.joint_state_publisher = self.node.create_publisher(
            JointState, '/joint_states', 10)
        self.node.get_logger().info('Controlador Webots do agrobot iniciado.')

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
        self.update_odometry(separation, now, stamp)
        self.publish_joint_states(stamp)
        if now >= self.next_scan_time:
            self.next_scan_time = now + self.lidar_period / 1000.0
            self.publish_scan(stamp)

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
