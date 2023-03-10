import math
import numpy as np

import rclpy
from rclpy.clock import Clock
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from locomotion.transforms import quaternion_from_euler

# ROS messages
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# PX4 autopilot ROS messages
from px4_msgs.msg import SensorCombined
from px4_msgs.msg import SensorGps
from px4_msgs.msg import VehicleAttitude

class MessageTranslation(Node):
	def __init__(self):
		super().__init__("MessageTranslation")

		# Set used parameters
		param_descriptor = ParameterDescriptor(description = "Sets the IMU topic name.")
		self.declare_parameter("imu_topic", "/imu/data", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the IMU frame ID.")
		self.declare_parameter("imu_frame_id", "imu", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the NavSatFix topic name.")
		self.declare_parameter("navsatfix_topic", "/gps/fix", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the NavSatFix frame ID.")
		self.declare_parameter("navsatfix_frame_id", "gps", param_descriptor)

		# Load parameters
		imu_topic = self.get_parameter("imu_topic").value
		self.imu_frame_id = self.get_parameter("imu_frame_id").value
		navsatfix_topic = self.get_parameter("navsatfix_topic").value
		self.navsatfix_frame_id = self.get_parameter("navsatfix_frame_id").value

		# QoS Profile that is compatible with PX4 (Important!!)
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,
			durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history = QoSHistoryPolicy.KEEP_LAST,
			depth = 1
		)

		# Init subscriber variables
		self.attitude_quaternion = np.array([0.0, 0.0, 0.0, 0.0])

		# Start Subscribers: Use PX4 uROS (dds) communication
		self.wheel_encoder_subscriber = self.create_subscription(
			SensorGps,
			"/fmu/out/vehicle_gps_position",
			self.vehicle_gps_position_callback_uros,
			qos_profile)
		self.sensor_combined_subscriber = self.create_subscription(
			SensorCombined,
			"/fmu/out/sensor_combined",
			self.sensor_combined_callback_uros,
			qos_profile)
		self.vehicle_attitude_subscriber = self.create_subscription(
			VehicleAttitude,
			"/fmu/out/vehicle_attitude",
			self.vehicle_attitude_callback_uros,
			qos_profile)

		# Start ROS Publishers
		self.imu_publisher = self.create_publisher(Imu, imu_topic, 10)
		self.navsatfix_publisher = self.create_publisher(NavSatFix, navsatfix_topic, 10)
		self.tf_imu_static_broadcast = StaticTransformBroadcaster(self)
		self.tf_gps_static_broadcast = StaticTransformBroadcaster(self)

		# Publish static transforms once at startup
		self.imu_transform()
		self.gps_transform()

	def imu_transform(self):
		tf = TransformStamped()

		tf.header.stamp = self.get_clock().now().to_msg()
		tf.header.frame_id = "base_link"
		tf.child_frame_id = self.imu_frame_id

		tf.transform.translation.x = 0.0
		tf.transform.translation.y = 0.0
		tf.transform.translation.z = 0.0
		quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
		tf.transform.rotation.x = quaternion[0]
		tf.transform.rotation.y = quaternion[1]
		tf.transform.rotation.z = quaternion[2]
		tf.transform.rotation.w = quaternion[3]

		self.tf_imu_static_broadcast.sendTransform(tf)

	def gps_transform(self):
		tf = TransformStamped()

		tf.header.stamp = self.get_clock().now().to_msg()
		tf.header.frame_id = "base_link"
		tf.child_frame_id = self.navsatfix_frame_id

		tf.transform.translation.x = 0.0
		tf.transform.translation.y = 0.0
		tf.transform.translation.z = 0.0
		quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
		tf.transform.rotation.x = quaternion[0]
		tf.transform.rotation.y = quaternion[1]
		tf.transform.rotation.z = quaternion[2]
		tf.transform.rotation.w = quaternion[3]

		self.tf_gps_static_broadcast.sendTransform(tf)

	# Callback function for new SensorGps message reception
	def vehicle_gps_position_callback_uros(self, msg):
		gps_msg = NavSatFix()
		gps_msg.header.stamp = self.get_clock().now().to_msg()
		# Set Frame IDs
		gps_msg.header.frame_id = self.navsatfix_frame_id
		# Set satellite fix status information
		if msg.fix_type == 0 or msg.fix_type == 1:
			gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
		elif msg.fix_type > 1:
			gps_msg.status.status = NavSatStatus.STATUS_FIX
		gps_msg.status.service = NavSatStatus.SERVICE_GPS + NavSatStatus.SERVICE_GLONASS + NavSatStatus.SERVICE_COMPASS + NavSatStatus.SERVICE_GALILEO
		# Set latitude, longitude, altitude
		gps_msg.latitude = msg.lat * 1.0E-7
		gps_msg.longitude = msg.lon * 1.0E-7
		gps_msg.altitude = msg.alt * 1.0E-3
		# Set covariance (Row-Major)
		gps_msg.position_covariance[0] = msg.eph
		gps_msg.position_covariance[1] = 0.0
		gps_msg.position_covariance[2] = 0.0
		gps_msg.position_covariance[3] = 0.0
		gps_msg.position_covariance[4] = msg.eph
		gps_msg.position_covariance[5] = 0.0
		gps_msg.position_covariance[6] = 0.0
		gps_msg.position_covariance[7] = 0.0
		gps_msg.position_covariance[8] = msg.epv
		# Set covariance type
		gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
		# Publish message
		self.navsatfix_publisher.publish(gps_msg)

	# Callback function for new VehicleAttitude message reception
	def vehicle_attitude_callback_uros(self, msg):
		# Get quaternion attitude
		self.attitude_quaternion[0] = msg.q[0]
		self.attitude_quaternion[1] = msg.q[1]
		self.attitude_quaternion[2] = msg.q[2]
		self.attitude_quaternion[3] = msg.q[3]

	# Callback function for new SensorCombined message reception
	def sensor_combined_callback_uros(self, msg):
		imu_msg = Imu()
		imu_msg.header.stamp = self.get_clock().now().to_msg()
		# Set Frame IDs
		imu_msg.header.frame_id = self.imu_frame_id
		# Set orientation
		imu_msg.orientation.x = self.attitude_quaternion[0]
		imu_msg.orientation.y = -self.attitude_quaternion[1]
		imu_msg.orientation.z = -self.attitude_quaternion[2]
		imu_msg.orientation.w = self.attitude_quaternion[3]
		# Set orientation covariance (Row-Major)
		# If orientation is not estimated, discard it by setting element 0 of cov. matrix to -1
		imu_msg.orientation_covariance[0] = 0.0
		imu_msg.orientation_covariance[1] = 0.0
		imu_msg.orientation_covariance[2] = 0.0
		imu_msg.orientation_covariance[3] = 0.0
		imu_msg.orientation_covariance[4] = 0.0
		imu_msg.orientation_covariance[5] = 0.0
		imu_msg.orientation_covariance[6] = 0.0
		imu_msg.orientation_covariance[7] = 0.0
		imu_msg.orientation_covariance[8] = 0.0
		# Set angular velocities
		# Convert coordinate systems from PX4 (FW, RIGHT, DOWN) to ROS (FW, LEFT, UP)
		imu_msg.angular_velocity.x = (float)(msg.gyro_rad[0])
		imu_msg.angular_velocity.y = -(float)(msg.gyro_rad[1])
		imu_msg.angular_velocity.z = -(float)(msg.gyro_rad[2])
		# Set angular velocities covariance (Row-Major)
		imu_msg.angular_velocity_covariance[0] = 0.0
		imu_msg.angular_velocity_covariance[1] = 0.0
		imu_msg.angular_velocity_covariance[2] = 0.0
		imu_msg.angular_velocity_covariance[3] = 0.0
		imu_msg.angular_velocity_covariance[4] = 0.0
		imu_msg.angular_velocity_covariance[5] = 0.0
		imu_msg.angular_velocity_covariance[6] = 0.0
		imu_msg.angular_velocity_covariance[7] = 0.0
		imu_msg.angular_velocity_covariance[8] = 0.0
		# Set linear acceleration
		# Convert coordinate systems from PX4 (FW, RIGHT, DOWN) to ROS (FW, LEFT, UP)
		imu_msg.linear_acceleration.x = (float)(msg.accelerometer_m_s2[0])
		imu_msg.linear_acceleration.y = -(float)(msg.accelerometer_m_s2[1])
		imu_msg.linear_acceleration.z = -(float)(msg.accelerometer_m_s2[2])
		# Set linear acceleration covariance (Row-Major)
		imu_msg.linear_acceleration_covariance[0] = 0.0
		imu_msg.linear_acceleration_covariance[1] = 0.0
		imu_msg.linear_acceleration_covariance[2] = 0.0
		imu_msg.linear_acceleration_covariance[3] = 0.0
		imu_msg.linear_acceleration_covariance[4] = 0.0
		imu_msg.linear_acceleration_covariance[5] = 0.0
		imu_msg.linear_acceleration_covariance[6] = 0.0
		imu_msg.linear_acceleration_covariance[7] = 0.0
		imu_msg.linear_acceleration_covariance[8] = 0.0
		# Publish message
		self.imu_publisher.publish(imu_msg)

def main(args=None):
	rclpy.init(args=args)

	print("Starting Message Translation...\n")

	message_translation = MessageTranslation()

	rclpy.spin(message_translation)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	message_translation.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()