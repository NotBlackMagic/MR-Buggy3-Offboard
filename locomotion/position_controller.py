import math
import numpy as np
from PyQt5 import QtWidgets
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import time
import collections

import rclpy
from rclpy.clock import Clock
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from locomotion.transforms import euler_from_quaternion

# ROS messages
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PositionController(Node):
	def __init__(self):
		super().__init__("PositionController")

		# Set used parameters
		param_descriptor = ParameterDescriptor(description = "Sets the subscribed Pose Stamped topic name.")
		self.declare_parameter("pose_topic", "goal_pose", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the subscribed Odometry topic name.")
		self.declare_parameter("odom_topic", "/wheel/odometry", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the published Twist topic name.")
		self.declare_parameter("twist_topic", "twist", param_descriptor)

		param_descriptor = ParameterDescriptor(description = "Sets the update rate (in Hz) of the position control PID loop.")
		self.declare_parameter("pid_loop_rate", 10.0, param_descriptor)

		# Load parameters
		pose_topic = self.get_parameter("pose_topic").value
		odom_topic = self.get_parameter("odom_topic").value
		twist_topic = self.get_parameter("twist_topic").value
		pid_loop_rate = self.get_parameter("pid_loop_rate").value

		# Init subscriber variables
		self.target_position = np.array([0.0, 0.0, 0.0])
		self.target_orientation = np.array([0.0, 0.0, 0.0, 0.0])
		self.current_position = np.array([0.0, 0.0, 0.0])
		self.current_orientation = np.array([0.0, 0.0, 0.0, 0.0])
		self.current_velocity_linear = np.array([0.0, 0.0, 0.0])
		self.current_velocity_angular = np.array([0.0, 0.0, 0.0])

		# self.target_position[0] = 100
		# self.target_position[1] = 100

		# Init control variables
		self.control_twist = np.array([0.0, 0.0, 0.0])
		self.control_angular = np.array([0.0, 0.0, 0.0])
		self.linear_threshold = 2.0					# Distance from target position that is considered as target reached
		self.proportional = 0.1						# Weight of distance on linear velocity (Proportional term in PID)
		self.max_linear_speed = 2.0					# Maximum forward speed in m/s
		self.max_angular_speed = math.radians(600)	# Maximum turn speed in rad/s (converted from deg/s)

		# Plotting
		self.plotting_en = True
		if(self.plotting_en):
			self.app = QtWidgets.QApplication([])
			self.graphWidget = pg.PlotWidget() # creates a window
			self.graphWidget.setTitle("Position Controller", size="20pt")
			self.graphWidget.setLabel("left", "Value")
			self.graphWidget.setLabel("bottom", "Call")
			self.graphWidget.showGrid(x=True, y=True)
			self.graphWidget.setYRange(-10, 10, padding=0)
			self.graphWidget.addLegend()

			# Plotting data holder
			window_width = 300
			self.dataDistance = np.linspace(0,0,window_width)
			self.dataAngle = np.linspace(0,0,window_width)
			self.dataLinear = np.linspace(0,0,window_width)
			self.dataAngular = np.linspace(0,0,window_width)
			self.ptr = -window_width

			# Create plot curves
			self.curveDistance = self.graphWidget.plot(name = "Distance (m)", pen=pg.mkPen(color=(255, 0, 0), width=2))
			self.curveAngle = self.graphWidget.plot(name = "Angle (rad)", pen=pg.mkPen(color=(0, 255, 0), width=2))
			self.curveLinear = self.graphWidget.plot(name = "Linear (m/s)", pen=pg.mkPen(color=(0, 0, 255), width=2))
			self.curveAngular = self.graphWidget.plot(name = "Angular (rad/s)", pen=pg.mkPen(color=(255, 255, 255), width=2))

			# Start plotting
			QtWidgets.QMainWindow.show(self.graphWidget)

		# Start ROS Subscribers
		self.pose_subscriber = self.create_subscription(
			PoseStamped,
			pose_topic,
			self.pose_callback,
			10)
		self.odom_subscriber = self.create_subscription(
			Odometry,
			odom_topic,
			self.odom_callback,
			10)

		# Start ROS Publishers
		self.publisher_twist = self.create_publisher(Twist, twist_topic, 10)

		# Start PID loop update timer
		timer_period = 1.0 / pid_loop_rate  # seconds
		self.timer = self.create_timer(timer_period, self.position_control_callback)
		self.pid_dt = timer_period

	# Callback function for new PoseStamped message reception
	def pose_callback(self, msg):
		self.target_position[0] = msg.pose.position.x
		self.target_position[1] = msg.pose.position.y
		self.target_position[2] = msg.pose.position.z
		self.target_orientation[0] = msg.pose.orientation.x
		self.target_orientation[1] = msg.pose.orientation.y
		self.target_orientation[2] = msg.pose.orientation.z
		self.target_orientation[3] = msg.pose.orientation.w

	# Callback function for new Odometry message reception
	def odom_callback(self, msg):
		self.current_position[0] = msg.pose.pose.position.x
		self.current_position[1] = msg.pose.pose.position.y
		self.current_position[2] = msg.pose.pose.position.z
		self.current_orientation[0] = msg.pose.pose.orientation.x
		self.current_orientation[1] = msg.pose.pose.orientation.y
		self.current_orientation[2] = msg.pose.pose.orientation.z
		self.current_orientation[3] = msg.pose.pose.orientation.w
		self.current_velocity_linear[0] = msg.twist.twist.linear.x
		self.current_velocity_linear[1] = msg.twist.twist.linear.y
		self.current_velocity_linear[2] = msg.twist.twist.linear.z
		self.current_velocity_angular[0] = msg.twist.twist.angular.x
		self.current_velocity_angular[1] = msg.twist.twist.angular.y
		self.current_velocity_angular[2] = msg.twist.twist.angular.z

	def position_control_callback(self):
		# Calculate movement vector: From current position to target position
		movement_vec = [self.target_position[0] - self.current_position[0],
		  				self.target_position[1] - self.current_position[1]]
		# Calculate current direction vector
		euler = euler_from_quaternion(	self.current_orientation[0],
										self.current_orientation[1],
										self.current_orientation[2],
										self.current_orientation[3])
		
		yaw = euler[2]
		current_vec = [	np.cos(yaw),
		 				np.sin(yaw)]
		# Calculate angle between vectors: movement and direction vectors
		# https://wumbo.net/formulas/angle-between-two-vectors-2d/
		cross = (movement_vec[1] * current_vec[0]) - (movement_vec[0] * current_vec[1])
		dot = (movement_vec[0] * current_vec[0]) + (movement_vec[1] * current_vec[1])
		angle = np.arctan2(cross, dot)
		# Calculate distance to be traveled
		distance = np.sqrt(movement_vec[0] * movement_vec[0] + movement_vec[1] * movement_vec[1])
		# Simple linear velocity controller, only P (proportional) term
		linear_speed = distance * self.proportional
		if(linear_speed > self.max_linear_speed):
			linear_speed = self.max_linear_speed
		# Calculate angular velocity based on linear velocity and angle to be turned in the travel time (travel in a curve)
		curve_radius = (distance / 2) / np.cos((np.pi / 2) - angle)
		curve_angle = 2 * angle
		curve_length = curve_radius * curve_angle
		travel_time = curve_length / linear_speed
		angular_speed = (2 * angle) / travel_time	# The rover has to travel twice the angle, so that end position is also tangent to travel curve/arc

		print("Target: X: %.2f, Y: %.2f; Current: X: %.2f, Y: %.2f, Yaw: %.2f\n" % (self.target_position[0], self.target_position[1], self.current_position[0], self.current_position[1], euler[2]))
		print("Distance (Straight): %.2f m; Distance (Curve): %.2f; Angle: %.2f rad\n" % (distance, curve_length, angle))
		print("Linear: %.2f m/s; Angular: %.2f rad/s\n" % (linear_speed, angular_speed))

		# Control movement
		if(np.abs(distance) > self.linear_threshold):
			if(distance > 0):
				# Move forward
				self.control_twist[0] = linear_speed
				# Turn
				self.control_angular[2] = angular_speed
			else:
				# Move backward
				self.control_twist[0] = -linear_speed
				# Turn
				self.control_angular[2] = angular_speed
		else:
			self.control_twist[0] = 0
			self.control_angular[2] = 0

		# Data plotting
		if(self.plotting_en):
			self.dataDistance[:-1] = self.dataDistance[1:]
			self.dataDistance[-1] = distance
			self.dataAngle[:-1] = self.dataAngle[1:]
			self.dataAngle[-1] = angle
			self.dataLinear[:-1] = self.dataLinear[1:]
			self.dataLinear[-1] = linear_speed
			self.dataAngular[:-1] = self.dataAngular[1:]
			self.dataAngular[-1] = angular_speed
			self.ptr += 1

			self.curveDistance.setData(self.dataDistance)
			self.curveDistance.setPos(self.ptr, 0)
			self.curveAngle.setData(self.dataAngle)
			self.curveAngle.setPos(self.ptr, 0)
			self.curveLinear.setData(self.dataLinear)
			self.curveLinear.setPos(self.ptr, 0)
			self.curveAngular.setData(self.dataAngular)
			self.curveAngular.setPos(self.ptr, 0)
			QtWidgets.QApplication.processEvents()
		
		# Publish
		self.publish_twist()

	def publish_twist(self):
		twist_msg = Twist()
		twist_msg.linear.x = self.control_twist[0]
		twist_msg.linear.y = self.control_twist[1]
		twist_msg.linear.z = self.control_twist[2]
		twist_msg.angular.x = self.control_angular[0]
		twist_msg.angular.y = self.control_angular[1]
		twist_msg.angular.z = self.control_angular[2]
		self.publisher_twist.publish(twist_msg)

def main(args=None):
	rclpy.init(args=args)

	print("Starting Position Controller...\n")

	position_controller = PositionController()

	# plt.ion()
	# plt.show()
	rclpy.spin(position_controller)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	position_controller.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()