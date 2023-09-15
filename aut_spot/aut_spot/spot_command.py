import time

import rclpy
from rclpy.node import Node

from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import robot_state_pb2
from bosdyn.api import geometry_pb2

from aut_msgs.msg import SpeedCommand

class SpotCommand(Node):

	def __init__(self):
		super().__init__("spot_command")

		# Paramerters

		# Connection
		self.declare_parameter("spot_ip", "192.168.80.3")
		self.spot_ip = self.get_parameter("spot_ip").get_parameter_value().string_value

		self.declare_parameter("spot_username", "user")
		self.spot_username = self.get_parameter("spot_username").get_parameter_value().string_value

		self.declare_parameter("spot_password", "upsa43jm7vnf")
		self.spot_password = self.get_parameter("spot_password").get_parameter_value().string_value

		self.get_logger().info('Connecting to Spot...')
		# Connect to Spot
		self.sdk = create_standard_sdk('DataClient')
		self.robot = self.sdk.create_robot(self.spot_ip)
		self.robot.authenticate(self.spot_username, self.spot_password)
		self.robot.sync_with_directory()
		self.robot.time_sync.wait_for_sync()
		self.get_logger().info('Connected')

		self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
		self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
		self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)

		self.lease = self.lease_client.take()
		self.lease_keepalive = LeaseKeepAlive(self.lease_client)

		self.get_logger().info('Acquire the lease')

		command_proto = RobotCommandBuilder.synchro_stand_command()
		self.command_client.robot_command(lease=None, command=command_proto, end_time_secs=None)

		self.subscription = self.create_subscription(SpeedCommand, "aut_local_planner/speed_command", self.listener_callback, 10)

		time.sleep(2)

	def listener_callback(self, msg):

		params = spot_command_pb2.MobilityParams(
			vel_limit=geometry_pb2.SE2VelocityLimit(max_vel=geometry_pb2.SE2Velocity(linear=geometry_pb2.Vec2(x=0.5, y=0.5), angular=0.4), min_vel=geometry_pb2.SE2Velocity(linear=geometry_pb2.Vec2(x=-0.5, y=-0.5), angular=-0.4)),
			locomotion_hint=spot_command_pb2.HINT_AUTO
		)

		end_time_secs = time.time() + 0.5
		transforms = self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
		command_proto = RobotCommandBuilder.synchro_velocity_command(msg.v_x, msg.v_y, msg.v_t, params=params)
		# command_proto = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(msg.x, msg.y, msg.angle, transforms, params=params)
		self.command_client.robot_command(lease=None, command=command_proto, end_time_secs=end_time_secs)


def main(args=None):
	rclpy.init(args=args)

	spot_command = SpotCommand()

	rclpy.spin(spot_command)

	spot_driver.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()