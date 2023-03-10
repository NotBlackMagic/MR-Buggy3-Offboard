# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Subsribed msgs by PX4 autopilot
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode

# Published msgs from PX4 autopilot
from px4_msgs.msg import SensorCombined
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleStatus

class AckermannController(Node):

	def __init__(self):
		super().__init__('AckermannController')
		# QoS Profile that is compatible with PX4 (Important!!)
		qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

		self.subscription_sensor_combined = self.create_subscription(
			SensorCombined,
			"/fmu/out/sensor_combined",
			self.sensor_combined_callback,
			qos_profile)
		
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
		self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
		self.publisher_manual_control = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_setpoint', qos_profile)

		timer_period = 0.2  # seconds
		self.timer = self.create_timer(timer_period, self.manual_control_callback)
		self.dt = timer_period

		self.manual_mode()
		self.arm()
		
		self.get_logger().info("Sensor Combined Listener Started")

	def sensor_combined_callback(self, msg):
		self.get_logger().info('T: "%d"' % msg.timestamp)

	def manual_control_callback(self):
		self.publish_manual_control(0.0, 0.0, 750.0, 0.0)

	# Arm the vehicle
	def arm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
		self.get_logger().info("Arm command send")

	# Disarm the vehicle
	def disarm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
		self.get_logger().info("Disarm command send")

	def manual_mode(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 217.0, 1.0)
		self.get_logger().info("Manual flight mode command send")

	# Send Manual Controls to Autopilot:
	# rcX: Vehicle X Movement (pitch), forward (for ROVER not used)
	# rcY: Vehicle Y Movement (roll), right (for ROVER is STEERING/SERVO)
	# rcTh: Vehicle Z Movement (thrust), up (for ROVER is FORWARD/THROTTLE)
	# rcR: Vehicle Rotation Movement (yaw), clockwise (for ROVER not used)
	def publish_manual_control(self, x, y, th, r):
		manual_control_msg = ManualControlSetpoint()
		manual_control_msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
		manual_control_msg.data_source = 0
		manual_control_msg.roll = y
		manual_control_msg.pitch = x
		manual_control_msg.yaw = r
		manual_control_msg.throttle = th

		self.publisher_manual_control.publish(manual_control_msg)

	'''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
	def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
		msg = VehicleCommand()
		msg.param1 = param1
		msg.param2 = param2
		msg.command = command		# command ID
		msg.target_system = 1		# system which should execute the command
		msg.target_component = 1	# component which should execute the command, 0 for all components
		msg.source_system = 1		# system sending the command
		msg.source_component = 1	# component sending the command
		msg.from_external = True
		msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
		self.vehicle_command_publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	print("Starting Locomotion...\n")

	ackermann_controller = AckermannController()

	rclpy.spin(ackermann_controller)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	ackermann_controller.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
