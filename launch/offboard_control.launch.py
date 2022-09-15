import os

import ament_index_python.packages

from launch import LaunchDescription
import launch_ros.actions

import yaml


def generate_launch_description():
	share_dir = ament_index_python.packages.get_package_share_directory('offboard_control')
	# Passing parameters to a composed node must be done via a dictionary of
	# key -> value pairs.  Here we read in the data from the configuration file
	# and create a dictionary of it that the ComposableNode will accept.
	params_file = os.path.join(share_dir, 'config', 'trajectoryParameters.yaml')
	with open(params_file, 'r') as f:
		params = yaml.safe_load(f)['offboard_control']['ros__parameters']
	return LaunchDescription([
		launch_ros.actions.Node(
			package='offboard_control',
			executable='offboard_control_quintic',
			output='screen',
			parameters=[params]
		),
	])