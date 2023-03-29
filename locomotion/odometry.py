import math
import numpy as np

import rclpy
from rclpy.clock import Clock
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from locomotion.transforms import quaternion_from_euler
from locomotion.ackermann import AckermannOdometry

# ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# PX4 autopilot ROS messages
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import WheelEncoders

class OdometryHandler(Node):
	def __init__(self):
		super().__init__("Odometry")

		# Set used parameters
		param_descriptor = ParameterDescriptor(description = "Sets the update and publish rate (in Hz) of the odometry message.")
		self.declare_parameter("odometry_rate", 10.0, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets if the odometry TF message should be published.")
		self.declare_parameter("publish_tf", True, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the Odometry topic name.")
		self.declare_parameter("odom_topic", "/wheel/odometry", param_descriptor)		
		param_descriptor = ParameterDescriptor(description = "Sets the Odometry frame ID.")
		self.declare_parameter("frame_id", "odom", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the Odometry child frame ID.")
		self.declare_parameter("child_frame_id", "base_link", param_descriptor)

		# Default values are for the MR-Buggy3
		param_descriptor = ParameterDescriptor(description = "Sets the wheel base (in m), distance between front and back wheels.")
		self.declare_parameter("wheel_base", 0.226, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the encoder ticks per full revolution.")
		self.declare_parameter("tick_per_revolution", 2, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the wheel circumference (in m) of the wheels with encoder.")
		self.declare_parameter("wheel_circumference", 0.23248, param_descriptor)
		# Steering scaling (from RC command to angle in rad): y = x * 314 urad / RC value
		param_descriptor = ParameterDescriptor(description = "Sets the steering scaling (in rad) from RC command to steering angle.")
		self.declare_parameter("steering_scaling", 0.000314, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the steering offset (in rad) from RC command to steering angle.")
		self.declare_parameter("steering_offset", 0.0, param_descriptor)
		# Thrust/speed scaling (from RC command to speed in m/s)): y = (x - 500) * s = x*s - 500*s
		param_descriptor = ParameterDescriptor(description = "Sets the thrust scaling (in m/s) from RC command to vehicle speed.")
		self.declare_parameter("thrust_scaling", 1.0, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the thrust offset (in m/s) from RC command to vehicle speed.")
		self.declare_parameter("thrust_offset", 0.0, param_descriptor)

		# Load parameters
		wheel_base = self.get_parameter("wheel_base").value
		tick_per_revolution = self.get_parameter("tick_per_revolution").value
		wheel_circumference = self.get_parameter("wheel_circumference").value
		steering_scaling = self.get_parameter("steering_scaling").value
		steering_offset = self.get_parameter("steering_offset").value
		thrust_scaling = self.get_parameter("thrust_scaling").value
		thrust_offset = self.get_parameter("thrust_offset").value
		odometry_rate = self.get_parameter("odometry_rate").value
		self.publish_tf = self.get_parameter("publish_tf").value
		odom_topic = self.get_parameter("odom_topic").value
		self.frame_id = self.get_parameter("frame_id").value
		self.child_frame_id = self.get_parameter("child_frame_id").value

		# QoS Profile that is compatible with PX4 (Important!!)
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,
			durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history = QoSHistoryPolicy.KEEP_LAST,
			depth = 1
		)

		# Init subscriber variables
		self.manual_control_dT = 0.0
		self.manual_control_timestamp = 0.0
		self.manual_control_steering = 0.0
		self.manual_control_throttle = 0.0
		self.wheel_encoder_dT = np.array([0.1, 0.1])
		self.wheel_encoder_timestamp = np.array([0, 0])
		self.wheel_encoder_position = np.array([0, 0])
		self.wheel_encoder_speed = np.array([0, 0])
		self.wheel_encoder_pulsePerRev = np.array([0, 0])

		# Start Subscribers: Use PX4 uROS (dds) communication
		self.wheel_encoder_br_subscriber = self.create_subscription(
			WheelEncoders,
			"/fmu/out/wheel_encoders_br",
			self.wheel_encoder_br_callback_uros,
			qos_profile)
		self.wheel_encoder_bl_subscriber = self.create_subscription(
			WheelEncoders,
			"/fmu/out/wheel_encoders_bl",
			self.wheel_encoder_bl_callback_uros,
			qos_profile)
		self.manual_control_subscriber = self.create_subscription(
			ManualControlSetpoint,
			"/fmu/in/manual_control_setpoint",
			self.manual_control_callback_uros,
			qos_profile)


		# Start ROS Publishers
		self.odometry_publisher = self.create_publisher(Odometry, odom_topic, 10)
		self.tf_odom_broadcaster = TransformBroadcaster(self)

		# Start and set-up Ackermann Odometry
		self.ackermann_odometry = AckermannOdometry(wheel_base)
		self.ackermann_odometry.setup_encoders(False, True, tick_per_revolution, wheel_circumference)
		self.ackermann_odometry.setup_rc_scaling(steering_offset, steering_scaling, thrust_offset, thrust_scaling)

		# Start Odometry calculation timer
		timer_period = 1.0 / odometry_rate	# seconds
		self.timer = self.create_timer(timer_period, self.publish_odometry)
		self.dt = timer_period

	# Callback function for new WheelEncoder message reception
	def wheel_encoder_br_callback_uros(self, msg):
		# Right wheel encoder values
		self.wheel_encoder_dT[0] = msg.timestamp - self.wheel_encoder_timestamp[0]
		self.wheel_encoder_timestamp[0] = msg.timestamp
		self.wheel_encoder_position[0] = msg.encoder_position
		self.wheel_encoder_speed[0] = msg.speed
		self.wheel_encoder_pulsePerRev[0] = msg.pulses_per_rev
		
	# Callback function for new WheelEncoder message reception
	def wheel_encoder_bl_callback_uros(self, msg):
		# Left wheel encoder values
		self.wheel_encoder_dT[1] = msg.timestamp - self.wheel_encoder_timestamp[1]
		self.wheel_encoder_timestamp[1] = msg.timestamp
		self.wheel_encoder_position[1] = msg.encoder_position
		self.wheel_encoder_speed[1] = msg.speed
		self.wheel_encoder_pulsePerRev[1] = msg.pulses_per_rev
		# Update Ackermann odometry with newly received values
		self.ackermann_odometry.update_using_encoder(
			self.manual_control_steering,
			self.manual_control_throttle,
			brVal=self.wheel_encoder_position[0], 
			blVal=self.wheel_encoder_position[1])

	# Callback funtion for new ManualControlSetpoint message reception
	def manual_control_callback_uros(self, msg):
		self.manual_control_dT = msg.timestamp - self.manual_control_timestamp
		self.manual_control_timestamp = msg.timestamp
		self.manual_control_steering = msg.roll
		self.manual_control_throttle = msg.throttle
		# Update Ackermann odometry with newly received values
		self.ackermann_odometry.update_using_encoder(
			self.manual_control_steering,
			self.manual_control_throttle,
			brVal=self.wheel_encoder_position[0], 
			blVal=self.wheel_encoder_position[1])

	# ROS Odometry message publisher function
	def publish_odometry(self):
		odom_msg = Odometry()
		odom_msg.header.stamp = self.get_clock().now().to_msg()
		# Set Frame IDs
		odom_msg.header.frame_id = self.frame_id
		odom_msg.child_frame_id = self.child_frame_id
		# Set pose position
		odom_msg.pose.pose.position.x = self.ackermann_odometry.pose_point[0]
		odom_msg.pose.pose.position.y = self.ackermann_odometry.pose_point[1]
		odom_msg.pose.pose.position.z = self.ackermann_odometry.pose_point[2]
		# Set pose orientation
		quaternion = quaternion_from_euler(	self.ackermann_odometry.pose_orientation[0], 
											self.ackermann_odometry.pose_orientation[1], 
											self.ackermann_odometry.pose_orientation[2])
		odom_msg.pose.pose.orientation.x = quaternion[0]
		odom_msg.pose.pose.orientation.y = quaternion[1]
		odom_msg.pose.pose.orientation.z = quaternion[2]
		odom_msg.pose.pose.orientation.w = quaternion[3]
		# Set linear twist
		odom_msg.twist.twist.linear.x = self.ackermann_odometry.twist_linear[0]
		odom_msg.twist.twist.linear.y = self.ackermann_odometry.twist_linear[1]
		odom_msg.twist.twist.linear.z = self.ackermann_odometry.twist_linear[2]
		# Set angular twist
		odom_msg.twist.twist.angular.x = self.ackermann_odometry.twist_angular[0]
		odom_msg.twist.twist.angular.y = self.ackermann_odometry.twist_angular[1]
		odom_msg.twist.twist.angular.z = self.ackermann_odometry.twist_angular[2]

		# print("Odom: X: %.2f, Y: %.2f, Yaw: %.2f\n" % (self.ackermann_odometry.pose_point[0], self.ackermann_odometry.pose_point[1], self.ackermann_odometry.pose_orientation[2]))

		# Publish
		self.odometry_publisher.publish(odom_msg)

		if self.publish_tf == True:
			# Set TF message for the odometry message
			tf2_msg = TransformStamped()
			tf2_msg.header.stamp = self.get_clock().now().to_msg()
			# Set Frame IDs
			tf2_msg.header.frame_id = self.frame_id
			tf2_msg.child_frame_id = self.child_frame_id
			# Set position transform between frames
			tf2_msg.transform.translation.x = self.ackermann_odometry.pose_point[0]
			tf2_msg.transform.translation.y = self.ackermann_odometry.pose_point[1]
			tf2_msg.transform.translation.z = self.ackermann_odometry.pose_point[2]
			# Set rotation transform between frames
			tf2_msg.transform.rotation.x = quaternion[0]
			tf2_msg.transform.rotation.y = quaternion[1]
			tf2_msg.transform.rotation.z = quaternion[2]
			tf2_msg.transform.rotation.w = quaternion[3]
			# Publish the transformation
			self.tf_odom_broadcaster.sendTransform(tf2_msg)

def main(args=None):
	rclpy.init(args=args)

	print("Starting Odometry...\n")

	odometry = OdometryHandler()

	rclpy.spin(odometry)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	odometry.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()