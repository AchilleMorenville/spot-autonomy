import numpy as np
import threading

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.api import world_object_pb2
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, ODOM_FRAME_NAME, BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.api import local_grid_pb2
from bosdyn.client.local_grid import LocalGridClient

from aut_msgs.msg import Fiducial
from aut_msgs.msg import LocalGrid

def get_numpy_data_type(local_grid_proto):
	"""Convert the cell format of the local grid proto to a numpy data type."""
	if local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT16:
		return np.uint16
	elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT16:
		return np.int16
	elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8:
		return np.uint8
	elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT8:
		return np.int8
	elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT64:
		return np.float64
	elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT32:
		return np.float32
	else:
		return None

def expand_data_by_rle_count(local_grid_proto, data_type=np.int16):
	"""Expand local grid data to full bytes data using the RLE count."""
	cells_pz = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
	cells_pz_full = []
	# For each value of rle_counts, we expand the cell data at the matching index
	# to have that many repeated, consecutive values.
	for i in range(0, len(local_grid_proto.local_grid.rle_counts)):
		for j in range(0, local_grid_proto.local_grid.rle_counts[i]):
			cells_pz_full.append(cells_pz[i])
	return np.array(cells_pz_full)

def unpack_grid(local_grid_proto):
	"""Unpack the local grid proto."""
	# Determine the data type for the bytes data.
	data_type = get_numpy_data_type(local_grid_proto.local_grid)
	if data_type is None:
		print("Cannot determine the dataformat for the local grid.")
		return None
	# Decode the local grid.
	if local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RAW:
		full_grid = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
	elif local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RLE:
		full_grid = expand_data_by_rle_count(local_grid_proto, data_type=data_type)
	else:
		# Return nothing if there is no encoding type set.
		return None
	# Apply the offset and scaling to the local grid.
	if local_grid_proto.local_grid.cell_value_scale == 0:
		return full_grid
	full_grid_float = full_grid.astype(np.float64)
	full_grid_float *= local_grid_proto.local_grid.cell_value_scale
	full_grid_float += local_grid_proto.local_grid.cell_value_offset
	return full_grid_float

def create_TransformStamped(bosdyn_transform, frame_id, child_frame_id, stamp):
	trans = TransformStamped()
	trans.header.stamp = stamp
	trans.header.frame_id = frame_id
	trans.child_frame_id = child_frame_id
	trans.transform.translation.x = bosdyn_transform.x
	trans.transform.translation.y = bosdyn_transform.y
	trans.transform.translation.z = bosdyn_transform.z
	trans.transform.rotation.x = bosdyn_transform.rot.x
	trans.transform.rotation.y = bosdyn_transform.rot.y
	trans.transform.rotation.z = bosdyn_transform.rot.z
	trans.transform.rotation.w = bosdyn_transform.rot.w
	return trans

def create_Odometry(bosdyn_transform, frame_id, child_frame_id, stamp):
	odom = Odometry()
	odom.header.stamp = stamp
	odom.header.frame_id = frame_id
	
	odom.child_frame_id = child_frame_id

	odom.pose.pose.position.x = bosdyn_transform.x
	odom.pose.pose.position.y = bosdyn_transform.y
	odom.pose.pose.position.z = bosdyn_transform.z

	odom.pose.pose.orientation.x = bosdyn_transform.rot.x
	odom.pose.pose.orientation.y = bosdyn_transform.rot.y
	odom.pose.pose.orientation.z = bosdyn_transform.rot.z
	odom.pose.pose.orientation.w = bosdyn_transform.rot.w

	return odom	

class SpotData(Node):

	def __init__(self):
		super().__init__("spot_data")

		self.connected_to_spot = False
		self.publishing_odom = False

		self.robot_state_client = None
		self.timer_odom = None
		self.timer_vision = None

		self.last_detection = {}

		# Paramerters

		# Connection
		self.declare_parameter("spot_ip", "192.168.80.3")
		self.spot_ip = self.get_parameter("spot_ip").get_parameter_value().string_value

		self.declare_parameter("spot_username", "user")
		self.spot_username = self.get_parameter("spot_username").get_parameter_value().string_value

		self.declare_parameter("spot_password", "upsa43jm7vnf")
		self.spot_password = self.get_parameter("spot_password").get_parameter_value().string_value

		# State
		self.declare_parameter("spot_state_frequency", 50)
		self.spot_state_frequency = self.get_parameter("spot_state_frequency").get_parameter_value().integer_value

		# Fiducials
		self.declare_parameter("spot_fiducials_frequency", 2)
		self.spot_fiducials_frequency = self.get_parameter("spot_fiducials_frequency").get_parameter_value().integer_value

		# LocalMap
		self.declare_parameter("spot_local_grid_frequency", 10)
		self.spot_local_grid_frequency = self.get_parameter("spot_local_grid_frequency").get_parameter_value().integer_value

		# Publishers
		self.ko_publisher = self.create_publisher(Odometry, "aut_spot/odometry/k_odom", 10)
		self.vo_publisher = self.create_publisher(Odometry, "aut_spot/odometry/v_odom", 10)

		self.fiducials_publisher = self.create_publisher(Fiducial, "aut_spot/fiducial", 10)
		self.local_grid_publisher = self.create_publisher(LocalGrid, "aut_spot/local_grid", 10)

		# TF2 Broadcaster
		self.tf_broadcaster = TransformBroadcaster(self)

		self.get_logger().info('Connecting to Spot...')
		# Connect to Spot
		self.sdk = create_standard_sdk('DataClient')
		self.robot = self.sdk.create_robot(self.spot_ip)
		self.robot.authenticate(self.spot_username, self.spot_password)
		self.robot.sync_with_directory()
		self.robot.time_sync.wait_for_sync()
		self.get_logger().info('Connected')

		# Launch odometries
		self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
		
		timer_period_state = 1.0 / self.spot_state_frequency
		self.timer_state = self.create_timer(timer_period_state, self.state_call_back)

		# Launch fiducials
		self.world_object_client = self.robot.ensure_client(WorldObjectClient.default_service_name)

		timer_period_fiducials = 1.0 / self.spot_fiducials_frequency
		self.timer_fiducials = self.create_timer(timer_period_fiducials, self.fiducials_call_back)

		# Launch LocalGrids
		self.local_grid_client = self.robot.ensure_client(LocalGridClient.default_service_name)

		timer_period_local_grids = 1.0 / self.spot_local_grid_frequency
		self.timer_local_grids = self.create_timer(timer_period_local_grids, self.local_grids_call_back)

		self.tf_broadcaster_lock = threading.Lock()

	def state_call_back(self):
		robot_state = self.robot_state_client.get_robot_state()

		stamp_robot = robot_state.kinematic_state.acquisition_timestamp
		seconds_real = stamp_robot.seconds - self.robot.time_sync.get_robot_clock_skew().seconds
		nanos_real = stamp_robot.nanos - self.robot.time_sync.get_robot_clock_skew().nanos

		if nanos_real < 0:
			nanos_real = nanos_real + 1_000_000_000
			seconds_real = seconds_real - 1

		stamp_real = Time()
		stamp_real.sec = seconds_real
		stamp_real.nanosec = nanos_real

		transforms_snapshot = robot_state.kinematic_state.transforms_snapshot

		# Kinematic odometry
		body_tform_odom = get_a_tform_b(
			transforms_snapshot,
			BODY_FRAME_NAME,
			ODOM_FRAME_NAME
		)

		# Vision odometry
		vision_tform_body = get_a_tform_b(
			transforms_snapshot,
			VISION_FRAME_NAME,
			BODY_FRAME_NAME
		)

		# Gravity
		body_tform_gravity = get_a_tform_b(
			transforms_snapshot,
			BODY_FRAME_NAME,
			"flat_body"
		)

		ko = create_TransformStamped(body_tform_odom, "base_link", "k_odom", stamp_real)
		vo = create_TransformStamped(vision_tform_body, "v_odom", "base_link", stamp_real)
		grav = create_TransformStamped(body_tform_gravity, "base_link", "gravity", stamp_real)

		# with self.tf_broadcaster_lock:
		self.tf_broadcaster.sendTransform(ko)
		self.tf_broadcaster.sendTransform(vo)
		self.tf_broadcaster.sendTransform(grav)

		# ko = create_Odometry(body_tform_odom, "k_odom", "base_link", stamp_real)
		# vo = create_Odometry(body_tform_vision, "v_odom", "base_link", stamp_real)
		# ko_publisher.publish(ko)
		# vo_publisher.publish(vo)
		
		# grav = create_TransformStamped(body_tform_gravity, "base_link", "gravity", stamp_real)
		# with self.tf_broadcaster_lock:
		# 	self.tf_broadcaster.sendTransform(grav)

		self.get_logger().info('Published odometries')

	def fiducials_call_back(self):

		# Get fiducials
		world_objects = self.world_object_client.list_world_objects(
			object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
		).world_objects

		stamp = self.get_clock().now().to_msg()

		for world_object in world_objects:

			if world_object.apriltag_properties.fiducial_pose_status != world_object_pb2.AprilTagProperties.AprilTagPoseStatus.STATUS_OK:
				continue

			timestamp = world_object.acquisition_time

			tag_id = world_object.apriltag_properties.tag_id

			# Sends only when new fiducial as been detected because fiducials are stored and reported during 15s after last appearance.
			# if tag_id in self.last_detection:
			# 	if timestamp.seconds - self.last_detection[tag_id].seconds == 0 and timestamp.nanos - self.last_detection[tag_id].nanos < 100:
			# 		continue
			# 	else:
			# 		self.last_detection[tag_id] = timestamp
			# else:
			# 	self.last_detection[tag_id] = timestamp

			seconds_real = timestamp.seconds - self.robot.time_sync.get_robot_clock_skew().seconds
			nanos_real = timestamp.nanos - self.robot.time_sync.get_robot_clock_skew().nanos

			if nanos_real < 0:
				nanos_real = nanos_real + 1_000_000_000
				seconds_real = seconds_real - 1

			stamp_real = Time()
			stamp_real.sec = seconds_real
			stamp_real.nanosec = nanos_real

			transforms_snapshot = world_object.transforms_snapshot

			body_tform_fiducial = get_a_tform_b(
				transforms_snapshot,
				BODY_FRAME_NAME,
				world_object.apriltag_properties.frame_name_fiducial_filtered
			)

			fiducial = Fiducial()

			fiducial.pose.translation.x = body_tform_fiducial.x
			fiducial.pose.translation.y = body_tform_fiducial.y
			fiducial.pose.translation.z = body_tform_fiducial.z

			fiducial.pose.rotation.x = body_tform_fiducial.rot.x
			fiducial.pose.rotation.y = body_tform_fiducial.rot.y
			fiducial.pose.rotation.z = body_tform_fiducial.rot.z
			fiducial.pose.rotation.w = body_tform_fiducial.rot.w

			fiducial.tag_id = tag_id
			fiducial.header.stamp = stamp_real
			fiducial.header.frame_id = "base_link"

			self.fiducials_publisher.publish(fiducial)

			fid = create_TransformStamped(body_tform_fiducial, "base_link", f"fiducial_{tag_id}", stamp_real)

			# with self.tf_broadcaster_lock:
			self.tf_broadcaster.sendTransform(fid)

			self.get_logger().info('Published fiducial')

	def local_grids_call_back(self):
		local_grid_protos = self.local_grid_client.get_local_grids(["obstacle_distance"])

		local_grid_transforms = local_grid_protos[0].local_grid.transforms_snapshot

		body_tform_grid = get_a_tform_b(
			local_grid_transforms,
			BODY_FRAME_NAME,
			local_grid_protos[0].local_grid.frame_name_local_grid_data
		)

		cells_obstacle_dist = unpack_grid(local_grid_protos[0]).astype(np.float32).tolist()

		timestamp = local_grid_protos[0].local_grid.acquisition_time

		seconds_real = timestamp.seconds - self.robot.time_sync.get_robot_clock_skew().seconds
		nanos_real = timestamp.nanos - self.robot.time_sync.get_robot_clock_skew().nanos

		if nanos_real < 0:
			nanos_real = nanos_real + 1_000_000_000
			seconds_real = seconds_real - 1

		stamp_real = Time()
		stamp_real.sec = seconds_real
		stamp_real.nanosec = nanos_real

		local_grid = LocalGrid()

		local_grid.header.stamp = stamp_real
		local_grid.header.frame_id = "base_link"
		local_grid.local_grid = cells_obstacle_dist

		local_grid.pose.translation.x = body_tform_grid.x
		local_grid.pose.translation.y = body_tform_grid.y
		local_grid.pose.translation.z = body_tform_grid.z

		local_grid.pose.rotation.x = body_tform_grid.rot.x
		local_grid.pose.rotation.y = body_tform_grid.rot.y
		local_grid.pose.rotation.z = body_tform_grid.rot.z
		local_grid.pose.rotation.w = body_tform_grid.rot.w

		self.local_grid_publisher.publish(local_grid)

		lg = create_TransformStamped(body_tform_grid, "base_link", "local_grid", stamp_real)

		# with self.tf_broadcaster_lock:
		self.tf_broadcaster.sendTransform(lg)

		self.get_logger().info('Published local grid')


def main(args=None):
	rclpy.init(args=args)

	spot_data = SpotData()

	rclpy.spin(spot_data)

	spot_driver.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
