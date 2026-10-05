"""
Run a custom YAML-defined scenario against a CARLA world that is already
running with the bridge as ticking master and the ego vehicle spawned (see
sim_bringup's ego_vehicle.launch.py).

This is the lightweight alternative to sim_scenarios' OpenSCENARIO +
scenario_runner path: no external project, just a YAML list of actors,
their spawn poses, and when each one should appear.

Usage:
  ros2 launch sim_scenarios_yaml run_yaml_scenario.launch.py
  ros2 launch sim_scenarios_yaml run_yaml_scenario.launch.py \
      scenario_yaml:=/abs/path/my_scenario.yaml

To write your own, copy scenarios/example_cone_scenario.yaml and edit it --
see that file and yaml_scenario_node.py's module docstring for the schema
and the three trigger types (immediate, time, ego_distance).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_scenario_file = os.path.join(
        get_package_share_directory('sim_scenarios_yaml'),
        'scenarios', 'example_cone_scenario.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('scenario_yaml', default_value=default_scenario_file),
        Node(
            package='sim_scenarios_yaml',
            executable='yaml_scenario',
            name='yaml_scenario',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'carla_host': LaunchConfiguration('host'),
                'carla_port': LaunchConfiguration('port'),
                'scenario_yaml': LaunchConfiguration('scenario_yaml'),
            }],
        ),
    ])
