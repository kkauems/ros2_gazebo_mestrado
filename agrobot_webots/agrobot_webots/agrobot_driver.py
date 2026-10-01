import math

import rclpy
from geometry_msgs.msg import Twist


class AgrobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.wheel_radius = 0.22
        self.wheel_separation = 0.96
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

        rclpy.init(args=None)
        self.node = rclpy.create_node('agrobot_driver')
        self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.node.get_logger().info('Controlador Webots do agrobot iniciado.')

    def get_motor(self, name):
        motor = self.robot.getDevice(name)
        motor.setPosition(math.inf)
        motor.setVelocity(0.0)
        return motor

    def cmd_vel_callback(self, message):
        self.linear_velocity = message.linear.x
        self.angular_velocity = message.angular.z

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

        left_speed = (
            self.linear_velocity
            - self.angular_velocity * self.wheel_separation / 2.0
        ) / self.wheel_radius
        right_speed = (
            self.linear_velocity
            + self.angular_velocity * self.wheel_separation / 2.0
        ) / self.wheel_radius

        for motor in self.left_motors:
            motor.setVelocity(left_speed)

        for motor in self.right_motors:
            motor.setVelocity(right_speed)
