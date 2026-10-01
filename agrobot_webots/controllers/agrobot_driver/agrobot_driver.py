#!/usr/bin/env python3

import math

import rclpy
from controller import Robot
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AgrobotDriver(Node):
	def __init__(self, robot):
		super().__init__('agrobot_driver')

		self.robot = robot
		self.time_step = int(robot.getBasicTimeStep())
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

		self.cmd_subscriber = self.create_subscription(
			Twist,
			'/cmd_vel',
			self.cmd_vel_callback,
			10,
		)

		self.get_logger().info('Controlador Webots do agrobot iniciado.')

	def get_motor(self, name):
		motor = self.robot.getDevice(name)
		motor.setPosition(math.inf)
		motor.setVelocity(0.0)
		return motor

	def cmd_vel_callback(self, message):
		self.linear_velocity = message.linear.x
		self.angular_velocity = message.angular.z

	def update_motors(self):
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

	def stop(self):
		for motor in self.left_motors + self.right_motors:
			motor.setVelocity(0.0)


def main():
	robot = Robot()
	rclpy.init()
	driver = AgrobotDriver(robot)

	try:
		while robot.step(driver.time_step) != -1 and rclpy.ok():
			rclpy.spin_once(driver, timeout_sec=0.0)
			driver.update_motors()
	finally:
		driver.stop()
		driver.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
