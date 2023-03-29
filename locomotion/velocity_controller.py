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

from locomotion.transforms import quaternion_from_euler
from locomotion.ackermann import AckermannOdometry
from locomotion.pid import PID

# ROS messages
from geometry_msgs.msg import Twist

# PX4 autopilot ROS messages
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import WheelEncoders

class VelocityController(Node):
	def __init__(self):
		super().__init__("VelocityController")

		# Set used parameters
		param_descriptor = ParameterDescriptor(description = "Sets the publish rate (in Hz) of the RC manual control message.")
		self.declare_parameter("manual_control_rate", 20.0, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the subscribed Twist topic name.")
		self.declare_parameter("twist_topic", "twist", param_descriptor)

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

		param_descriptor = ParameterDescriptor(description = "Sets the update rate (in Hz) of the RC manual control PID loop.")
		self.declare_parameter("pid_loop_rate", 10.0, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the thrust PID proportional gain.")
		self.declare_parameter("throttle_pid_kp", 0.01, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the thrust PID integral gain.")
		self.declare_parameter("throttle_pid_ki", 0.015, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the thrust PID derivative gain.")
		self.declare_parameter("throttle_pid_kd", 0.001, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the thrust PID integral windup limit.")
		self.declare_parameter("throttle_pid_lim", 1.0, param_descriptor)

		# Load parameters
		wheel_base = self.get_parameter("wheel_base").value
		tick_per_revolution = self.get_parameter("tick_per_revolution").value
		wheel_circumference = self.get_parameter("wheel_circumference").value
		steering_scaling = self.get_parameter("steering_scaling").value
		steering_offset = self.get_parameter("steering_offset").value
		thrust_scaling = self.get_parameter("thrust_scaling").value
		thrust_offset = self.get_parameter("thrust_offset").value
		manual_control_rate = self.get_parameter("manual_control_rate").value
		twist_topic = self.get_parameter("twist_topic").value
		pid_loop_rate = self.get_parameter("pid_loop_rate").value
		throttle_pid_kp = self.get_parameter("throttle_pid_kp").value
		throttle_pid_ki = self.get_parameter("throttle_pid_ki").value
		throttle_pid_kd = self.get_parameter("throttle_pid_kd").value
		throttle_pid_lim = self.get_parameter("throttle_pid_lim").value

		# QoS Profile that is compatible with PX4 (Important!!)
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,
			durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history = QoSHistoryPolicy.KEEP_LAST,
			depth = 1
		)

		# Init subscriber variables
		self.brEnc = 0
		self.blEnc = 0
		self.linear = np.array([0.0, 0.0, 0.0])
		self.angular = np.array([0.0, 0.0, 0.0])
		self.manual_control_steering = 0.0
		self.manual_control_throttle = 0.0

		# Init Velocity Controller PID variables
		self.throttle_pid = PID(throttle_pid_kp, throttle_pid_ki, throttle_pid_kd, throttle_pid_lim)
		# self.steering_pid = PID()

		# Plotting
		self.plotting_en = True
		if(self.plotting_en):
			self.app = QtWidgets.QApplication([])
			self.graphWidget = pg.PlotWidget() # creates a window
			self.graphWidget.setTitle("Velocity PID Controller", size="20pt")
			self.graphWidget.setLabel("left", "Value")
			self.graphWidget.setLabel("bottom", "Call")
			self.graphWidget.showGrid(x=True, y=True)
			self.graphWidget.setYRange(-1, 1, padding=0)
			self.graphWidget.addLegend()

			# Plotting data holder
			window_width = 300
			self.dataPID_p = np.linspace(0,0,window_width)
			self.dataPID_i = np.linspace(0,0,window_width)
			self.dataPID_d = np.linspace(0,0,window_width)
			self.dataPID_out = np.linspace(0,0,window_width)
			self.data_linear = np.linspace(0,0,window_width)
			self.data_angular = np.linspace(0,0,window_width)
			self.ptr = -window_width

			# Create plot curves
			self.curvePID_p = self.graphWidget.plot(name = "PID P", pen=pg.mkPen(color=(255, 0, 0), width=2))
			self.curvePID_i = self.graphWidget.plot(name = "PID I", pen=pg.mkPen(color=(0, 255, 0), width=2))
			self.curvePID_d = self.graphWidget.plot(name = "PID D", pen=pg.mkPen(color=(0, 0, 255), width=2))
			self.curvePID_out = self.graphWidget.plot(name = "PID Out", pen=pg.mkPen(color=(255, 255, 255), width=2))
			self.curve_linear = self.graphWidget.plot(name = "Speed (m/s)*0.1", pen=pg.mkPen(color=(255, 128, 0), width=2))
			self.curve_angular = self.graphWidget.plot(name = "Angular (rad/s)*0.1", pen=pg.mkPen(color=(255, 128, 0), width=2))

			# Start plotting
			QtWidgets.QMainWindow.show(self.graphWidget)

		# Start ROS Subscribers
		self.twist_subscriber = self.create_subscription(
			Twist,
			twist_topic,
			self.twist_callback,
			10)

		# Use PX4 uROS (dds) communication
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

		# Start ROS Publishers
		self.publisher_manual_control = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_setpoint', qos_profile)
		
		# Start and set-up Ackermann Odometry
		self.ackermann_odometry = AckermannOdometry(wheel_base)
		self.ackermann_odometry.setup_encoders(False, True, tick_per_revolution, wheel_circumference)
		self.ackermann_odometry.setup_rc_scaling(steering_offset, steering_scaling, thrust_offset, thrust_scaling)

		# Start manual control publisher timer
		timer_period = 1.0 / manual_control_rate  # seconds
		self.timer = self.create_timer(timer_period, self.manual_control_callback)
		self.dt = timer_period

		# Start PID loop update timer
		timer_period = 1.0 / pid_loop_rate  # seconds
		self.timer = self.create_timer(timer_period, self.velocity_control_callback)
		self.pid_dt = timer_period

	# Callback function for new WheelEncoder message reception
	def wheel_encoder_br_callback_uros(self, msg):
		# Right wheel encoder values
		self.brEnc = msg.encoder_position
		
		# self.ackermann_odometry.update_using_encoder(
		# 	self.manual_control_steering,
		# 	self.manual_control_throttle,
		# 	brVal=self.brEnc, 
		# 	blVal=self.blEnc)

	# Callback function for new WheelEncoder message reception
	def wheel_encoder_bl_callback_uros(self, msg):
		# Left wheel encoder values
		self.blEnc = msg.encoder_position

		if self.blEnc > self.brEnc:
			self.brEnc = self.blEnc
		else:
			self.blEnc = self.brEnc
		
		self.ackermann_odometry.update_using_encoder(
			self.manual_control_steering,
			self.manual_control_throttle,
			brVal=self.brEnc, 
			blVal=self.blEnc)
		# print("Cnt R: %02f; Cnt L:%02f" % ((float)(self.ackermann_odometry.encoder_count[2]), (float)(self.ackermann_odometry.encoder_count[3])))
		# print("RPS R: %02f; RPS L:%02f" % ((float)(self.ackermann_odometry.rps[2]), (float)(self.ackermann_odometry.rps[3])))
		# print("Speed (m/s): %02f" % self.ackermann_odometry.twist_linear[0])

	# Callback function for new Twist message reception
	def twist_callback(self, msg):
		self.linear[0] = msg.linear.x
		self.linear[1] = msg.linear.y
		self.linear[2] = msg.linear.z
		self.angular[0] = msg.angular.x
		self.angular[1] = msg.angular.y
		self.angular[2] = msg.angular.z

	def velocity_control_callback(self):
		# PID Controller for throttle
		throttle_ctrl = self.throttle_pid.PIDUpdate(self.linear[0], self.ackermann_odometry.twist_linear[0], self.pid_dt)

		# PID Controller for steering (for now direct calculation)
		steering_angle = self.ackermann_odometry.steering_angle_from_velocity(self.linear[0], self.angular[2])
		steering_ctrl = self.ackermann_odometry.angle_to_rc_command_steering(steering_angle)		
		# steering_ctrl = self.steering_pid.PIDUpdate(self.angular[2], self.ackermann_odometry.twistAngular[2], self.dt)

		# print("RC Thrust: %.3f, Steering: %.3f rad | %.0f\n" % (throttle_ctrl, steering_angle, steering_ctrl))

		# Scale values
		self.manual_control_throttle = (500.0 + throttle_ctrl * 500.0)
		# self.manual_control_steering = (steering_ctrl * 1000.0)
		self.manual_control_steering = steering_ctrl

		# Limit values
		if(self.manual_control_throttle > 1000):
			self.manual_control_throttle = 1000.0
		elif(self.manual_control_throttle < 0):
			self.manual_control_throttle = 0.0
		if(self.manual_control_steering > 1000):
			self.manual_control_steering = 1000.0
		elif(self.manual_control_steering < -1000):
			self.manual_control_steering = -1000.0

		# Data plotting
		if(self.plotting_en):
			self.dataPID_p[:-1] = self.dataPID_p[1:]
			self.dataPID_p[-1] = self.throttle_pid.pid_p
			self.dataPID_i[:-1] = self.dataPID_i[1:]
			self.dataPID_i[-1] = self.throttle_pid.pid_i
			self.dataPID_d[:-1] = self.dataPID_d[1:]
			self.dataPID_d[-1] = self.throttle_pid.pid_d
			self.dataPID_out[:-1] = self.dataPID_out[1:]
			self.dataPID_out[-1] = self.throttle_pid.pid_out
			self.data_linear[:-1] = self.data_linear[1:]
			self.data_linear[-1] = self.ackermann_odometry.twist_linear[0] / 10.0
			self.data_angular[:-1] = self.data_angular[1:]
			self.data_angular[-1] = self.ackermann_odometry.twist_angular[2] / 10.0
			self.ptr += 1

			self.curvePID_p.setData(self.dataPID_p)
			self.curvePID_p.setPos(self.ptr, 0)
			self.curvePID_i.setData(self.dataPID_i)
			self.curvePID_i.setPos(self.ptr, 0)
			self.curvePID_d.setData(self.dataPID_d)
			self.curvePID_d.setPos(self.ptr, 0)
			self.curvePID_out.setData(self.dataPID_out)
			self.curvePID_out.setPos(self.ptr, 0)
			self.curve_linear.setData(self.data_linear)
			self.curve_linear.setPos(self.ptr, 0)
			self.curve_angular.setData(self.data_angular)
			self.curve_angular.setPos(self.ptr, 0)
			QtWidgets.QApplication.processEvents()
		
		# print("PID: E: %02f; P: %02f; I: %02f; D: %02f; Out: %02f" % (self.throttle_pid.prev_error, self.throttle_pid.pid_p, self.throttle_pid.pid_i, self.throttle_pid.pid_d, self.manual_control_throttle))

	def manual_control_callback(self):
		self.publish_manual_control(0.0, self.manual_control_steering, self.manual_control_throttle, 0.0)

	def publish_manual_control(self, x: float, y: float, th: float, r: float):
		# Send Manual Controls to Autopilot:
		# rcX: Vehicle X Movement (pitch), forward (for ROVER not used)
		# rcY: Vehicle Y Movement (roll), right (for ROVER is STEERING/SERVO)
		# rcTh: Vehicle Z Movement (thrust), up (for ROVER is FORWARD/THROTTLE)
		# rcR: Vehicle Rotation Movement (yaw), clockwise (for ROVER not used)
		# Use PX4 uROS (dds) communication
		manual_control_msg = ManualControlSetpoint()
		manual_control_msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
		manual_control_msg.data_source = 0
		manual_control_msg.roll = y
		manual_control_msg.pitch = x
		manual_control_msg.yaw = r
		manual_control_msg.throttle = th
		self.publisher_manual_control.publish(manual_control_msg)

def main(args=None):
	rclpy.init(args=args)

	print("Starting Velocity Controller...\n")

	velocity_controller = VelocityController()

	# plt.ion()
	# plt.show()
	rclpy.spin(velocity_controller)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	velocity_controller.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()