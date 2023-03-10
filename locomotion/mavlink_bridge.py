import math
import numpy as np

import rclpy
from rclpy.clock import Clock
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from pymavlink import mavutil

from locomotion.transforms import quaternion_from_euler

# PX4 autopilot ROS messages
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import SensorCombined
from px4_msgs.msg import SensorGps
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import WheelEncoders

class MAVLinkBridge(Node):
	def __init__(self):
		super().__init__("MAVLinkBridge")

		# For Linux to PX4 Hardware via UART: "/dev/ttyUSB0", baud=57600 or "/dev/ttyACM0", baud=57600 
		# For Windows to PX4 Hardware via UART: "COM5", baud=57600
		param_descriptor = ParameterDescriptor(description = "Sets the MAVLink serial link port.")
		self.declare_parameter("mavlink_port", "/dev/ttyACM0", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the MAVLink serial link baudrate.")
		self.declare_parameter("mavlink_baud", 921600, param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the MAVLink message check/refresh rate.")
		self.declare_parameter("mavlink_rate", 250.0, param_descriptor)

		mavlink_port = self.get_parameter("mavlink_port").value
		mavlink_baud = self.get_parameter("mavlink_baud").value
		mavlink_rate = self.get_parameter("mavlink_rate").value

		# QoS Profile that is compatible with PX4 (Important!!)
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,
			durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history = QoSHistoryPolicy.KEEP_LAST,
			depth = 1
		)

		# Start ROS Publishers
		self.publisher_sensor_combined = self.create_publisher(SensorCombined, "/fmu/out/sensor_combined", qos_profile)
		self.publisher_vehicle_attitude = self.create_publisher(VehicleAttitude, "/fmu/out/vehicle_attitude", qos_profile)
		self.publisher_vehicle_gps = self.create_publisher(SensorGps, "/fmu/out/vehicle_gps_position", qos_profile)
		self.publisher_wheel_encoder_br = self.create_publisher(WheelEncoders, "/fmu/out/wheel_encoders_br", qos_profile)
		self.publisher_wheel_encoder_bl = self.create_publisher(WheelEncoders, "/fmu/out/wheel_encoders_bl", qos_profile)

		# Start ROS Subscribers
		self.manual_control_subscriber = self.create_subscription(
				ManualControlSetpoint,
				"/fmu/in/manual_control_setpoint",
				self.manual_control_callback_uros,
				qos_profile)
		self.vehicle_command_subscriber = self.create_subscription(
				VehicleCommand,
				"/fmu/in/vehicle_command",
				self.vehicle_command_callback_uros,
				qos_profile)				

		# Use PX4 MAVLink communication
		# Create MAVLink connection
		# Via UART/Serial:
		self.master = mavutil.mavlink_connection(mavlink_port, baud = mavlink_baud)
		# For Linux to PX4 Software via TCP
		# self.master = mavutil.mavlink_connection("udpin:127.0.0.1:14540", baud=57600)

		# Start MAVLink Subscribers (have to be checked regularly)
		timer_period = 1.0 / mavlink_rate	# seconds
		self.mavlink_timer = self.create_timer(timer_period, self.mavlink_callback)
		self.mavlink_dt = timer_period

	# Callback function for new ManualControlSetpoint message reception
	def manual_control_callback_uros(self, msg):
		# PX4 uROS message to MAVLink message
		#msg.timestamp
		#msg.timestamp_sample
		#msg.valid
		#msg.data_source = msg.data_source
		#msg.flaps
		#msg.aux1
		#msg.aux2
		#msg.aux3
		#msg.aux4
		#msg.aux5
		#msg.aux6
		#msg.sticks_moving
		self.master.mav.manual_control_send(
			self.master.target_system,	#Target system
			(int)(msg.pitch),			#X-Axis control input [-1000, 1000] (vehicle pitch, for ROVER not used)
			(int)(msg.roll),			#Y-Axis control input [-1000, 1000] (vehicle roll, for ROVER is STEERING/SERVO)
			(int)(msg.throttle),		#Z-Axis control input [-1000, 1000] or for legacy (in PX4) [0, 1000] (vehicle thrust, for ROVER is FORWARD/THROTTLE)
			(int)(msg.yaw),				#R-Axis normalized control input [-1000, 1000] (vehicle rotation/yaw, for ROVER not used)
			(int)(0))						#Buttons bit-field

	# Callback function for new VehicleCommand message reception
	def vehicle_command_callback_uros(self, msg):
		# PX4 uROS message to MAVLink message
		self.master.mav.command_long_send(
			msg.target_system,			# system which should execute the command
			msg.target_component,		# component which should execute the command, 0 for all components
			msg.command,				# command ID
			0, msg.param1, msg.param2, 0, 0, 0, 0, 0)
		# msg.timestamp
		# msg.source_system		# system sending the command
		# msg.source_component	# component sending the command
		# msg.from_external

	def attitude_callback_mavlink(self, name, msg):
		att_msg = VehicleAttitude()
		att_msg.timestamp = msg.time_boot_ms
		# att_msg.timestamp_sample = 0
		quaternion = quaternion_from_euler(
			msg.roll, 
			msg.pitch,
			msg.yaw)
		att_msg.q[0] = quaternion[0]
		att_msg.q[1] = quaternion[1]
		att_msg.q[2] = quaternion[2]
		att_msg.q[3] = quaternion[3]
		quaternion = quaternion_from_euler(
			msg.rollspeed, 
			msg.pitchspeed,
			msg.yawspeed)
		att_msg.delta_q_reset[0] = quaternion[0]
		att_msg.delta_q_reset[1] = quaternion[1]
		att_msg.delta_q_reset[2] = quaternion[2]
		att_msg.delta_q_reset[3] = quaternion[3]
		# att_msg.quat_reset_counter = 0
		self.publisher_vehicle_attitude.publish(att_msg)
		# if msg.get_type() == "ATTITUDE_QUATERNION":
		# 	# Get quaternion attitude
		#	att_msg = VehicleAttitude()
		# 	att_msg.q[0] = msg.q1
		# 	att_msg.q[1] = msg.q2
		# 	att_msg.q[2] = msg.q3
		# 	att_msg.q[3] = msg.q4
		#	self.publisher_vehicle_attitude.publish(att_msg)

	def gps_raw_callback_mavlink(self, name, msg):
		gps_msg = SensorGps()
		gps_msg.timestamp = msg.time_usec
		# gps_msg.timestamp_sample = 0
		# gps_msg.device_id = 0
		gps_msg.lat = msg.lat
		gps_msg.lon = msg.lon
		gps_msg.alt = msg.alt
		gps_msg.alt_ellipsoid = msg.alt_ellipsoid
		gps_msg.s_variance_m_s = msg.vel_acc / 1000
		# gps_msg.c_variance_rad = 0.0
		gps_msg.fix_type = msg.fix_type
		gps_msg.eph = msg.h_acc / 1000
		gps_msg.epv = msg.v_acc / 1000
		gps_msg.hdop = msg.eph / 100
		gps_msg.vdop = msg.epv / 100
		# gps_msg.noise_per_ms = 0
		# gps_msg.automatic_gain_control = 0
		# gps_msg.jamming_state = 0
		# gps_msg.jamming_indicator = 0
		gps_msg.vel_m_s = msg.vel / 100
		# gps_msg.vel_n_m_s = 0.0
		# gps_msg.vel_e_m_s = 0.0
		# gps_msg.vel_d_m_s = 0.0
		gps_msg.cog_rad = msg.cog * 0.0174533 / 100
		# gps_msg.vel_ned_valid = False
		# gps_msg.timestamp_time_relative = 0
		# gps_msg.time_utc_usec = 0
		gps_msg.satellites_used = msg.satellites_visible
		gps_msg.heading = msg.yaw * 0.0174533 / 100
		# gps_msg.heading_offset = 0
		gps_msg.heading_accuracy = msg.hdg_acc * 0.0174533 / 10000
		# gps_msg.rtcm_injection_rate = 0
		# gps_msg.selected_rtcm_instance = 0
		self.publisher_vehicle_gps.publish(gps_msg)

	def highres_imu_callback_mavlink(self, name, msg):
		imu_msg = SensorCombined()
		imu_msg.timestamp = msg.time_usec
		imu_msg.gyro_rad[0] = msg.xgyro
		imu_msg.gyro_rad[1] = msg.ygyro
		imu_msg.gyro_rad[2] = msg.zgyro
		# imu_msg.gyro_integral_dt = 0
		# imu_msg.accelerometer_timestamp_relative = 0
		imu_msg.accelerometer_m_s2[0] = msg.xacc
		imu_msg.accelerometer_m_s2[1] = msg.yacc
		imu_msg.accelerometer_m_s2[2] = msg.zacc
		# imu_msg.accelerometer_integral_dt = 0
		# imu_msg.accelerometer_clipping = 0
		# imu_msg.gyro_clipping = 0
		# imu_msg.accel_calibration_count = 0
		# imu_msg.gyro_calibration_count = 0
		self.publisher_sensor_combined.publish(imu_msg)

	def wheel_distance_callback_mavlink(self, name, msg):
		encoder_cnt = msg.count
		# Left wheel encoder values
		encoder_msg_br = WheelEncoders()
		encoder_msg_bl = WheelEncoders()
		encoder_msg_br.timestamp = msg.time_usec
		encoder_msg_bl.timestamp = msg.time_usec
		encoder_msg_br.encoder_position = (int)(msg.distance[0])
		encoder_msg_bl.encoder_position = (int)(msg.distance[1])
		# encoder_msg_br.speed = 0.0
		# encoder_msg_bl.speed = 0.0
		# encoder_msg_br.pulses_per_rev = 0.0
		# encoder_msg_bl.pulses_per_rev = 0.0
		self.publisher_wheel_encoder_br.publish(encoder_msg_br)
		self.publisher_wheel_encoder_bl.publish(encoder_msg_bl)

	# Callback function for MAVLink message check timer
	def mavlink_callback(self):
		# MAVLink messages to PX4 uROS messages
		msg = self.master.recv_match()
		if not msg:
			return
		if msg.get_type() == "ATTITUDE":
			self.attitude_callback_mavlink("ATTITUDE", msg)
		if msg.get_type() == "GPS_RAW_INT":
			self.gps_raw_callback_mavlink("GPS_RAW_INT", msg)
		if msg.get_type() == "HIGHRES_IMU":
			self.highres_imu_callback_mavlink("HIGHRES_IMU", msg)
		if msg.get_type() == "WHEEL_DISTANCE":
			self.wheel_distance_callback_mavlink("WHEEL_DISTANCE", msg)


def main(args=None):
	rclpy.init(args=args)

	print("Starting MAVLink Bridge...\n")

	mavlink_bridge = MAVLinkBridge()

	rclpy.spin(mavlink_bridge)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	mavlink_bridge.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()