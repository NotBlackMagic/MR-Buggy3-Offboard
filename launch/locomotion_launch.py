import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Set the path to different files and folders.
	robot_localization_config = os.path.join(
		get_package_share_directory("locomotion"), 
		"config",
		"ekf.yaml")

	# Start MAVLink Bridge
	start_mavlink_bridge = Node(
		package="locomotion",
		executable="mavlink_bridge",
		name="mavlink_bridge"
	)

	# Start odometry 
	start_odometry_cmd = Node(
		package="locomotion",
		executable="odometry",
		name="odometry"
	)

    # Start Velocity controller
	start_vel_ctrl_cmd = Node(
		package="locomotion",
		executable="vel_ctrl",
		name="vel_ctrl"
	)

	# Start Position controller
	start_pos_ctrl_cmd = Node(
		package="locomotion",
		executable="pos_ctrl",
		name="pos_ctrl"
	)

	# Start Keyboard Control 
	start_keyboard_ctrl_cmd = Node(
		package="locomotion",
		executable="keyboard_ctrl",
		name="keyboard_ctrl"
	)

    # Start message translation 
	start_msg_trans_cmd = Node(
		package="locomotion",
		executable="msg_trans",
		name="msg_trans"
	)

    # Start robot localization using an Extended Kalman filter
	start_robot_localization_local_cmd = Node(
		package="robot_localization",
		executable="ekf_node",
		name="ekf_filter_node",
		output="screen",
		parameters=[robot_localization_config],
		remappings=[('odometry/filtered', 'odometry/local')])

    # Start the navsat transform node which converts GPS data into the world coordinate frame
	start_navsat_transform_cmd = Node(
		package='robot_localization',
		executable='navsat_transform_node',
		name='navsat_transform',
		output='screen',
		parameters=[robot_localization_config],
		remappings=[('imu', 'imu/data'),
					('odometry/filtered', 'odometry/global')])

	# Start robot localization using an Extended Kalman filter...map->odom transform
	start_robot_localization_global_cmd = Node(
		package='robot_localization',
		executable='ekf_node',
		name='ekf_filter_node_map',
		output='screen',
		parameters=[robot_localization_config],
		remappings=[('odometry/filtered', 'odometry/global')])

    # Create the launch description and populate
	ld = LaunchDescription()

    # Declare the launch options

    # Add any actions
	ld.add_action(start_mavlink_bridge)
	ld.add_action(start_odometry_cmd)
	ld.add_action(start_vel_ctrl_cmd)
	# ld.add_action(start_pos_ctrl_cmd)
	# ld.add_action(start_keyboard_ctrl_cmd)
	ld.add_action(start_msg_trans_cmd)
	# ld.add_action(start_robot_localization_local_cmd)
	# ld.add_action(start_navsat_transform_cmd)
	# ld.add_action(start_robot_localization_global_cmd)

    #ros2 service call /datum robot_localization/srv/SetDatum '{geo_pose: {position: {latitude: 38.77, longitude: -9.16, altitude: 133.78}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
	ld.add_action(
		ExecuteProcess(
			cmd=[[
				FindExecutable(name='ros2'),
				" service call ",
				"/datum ",
				"obot_localization/srv/SetDatum ",
				'"{geo_pose: {position: {latitude: 38.77, longitude: -9.16, altitude: 133.78}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"',
			]],
			shell=True
		)
	)

	return ld