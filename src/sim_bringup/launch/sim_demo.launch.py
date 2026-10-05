"""
Full demo: bridge + lidar-only ego vehicle + seeded background traffic +
G29 wheel control, optionally followed by a custom scenario.

  ros2 launch sim_bringup sim_demo.launch.py
  ros2 launch sim_bringup sim_demo.launch.py seed:=42 num_vehicles:=20
  ros2 launch sim_bringup sim_demo.launch.py with_scenario:=true
  ros2 launch sim_bringup sim_demo.launch.py with_yaml_scenario:=true

Start a CARLA server first. The ego vehicle will not move in synchronous
mode until the G29 node (or some other controller) starts publishing to
/carla/<role_name>/vehicle_control_cmd -- that is expected.

Two independent, mutually compatible scenario options -- pick whichever
fits, or neither:

  with_scenario:=true       runs sim_scenarios' bundled OpenSCENARIO example
                            (FollowLeadingVehicle.xosc) via scenario_runner.
  with_yaml_scenario:=true  runs sim_scenarios_yaml's bundled native YAML
                            example (example_cone_scenario.yaml), no
                            scenario_runner required.

For a custom file with either, launch that package's own run_*.launch.py
directly instead of using these two shortcuts, which only run the bundled
example.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('town', default_value='Town01'),
        DeclareLaunchArgument('role_name', default_value='ego_vehicle'),
        DeclareLaunchArgument('seed', default_value='1234'),
        DeclareLaunchArgument('num_vehicles', default_value='30'),
        DeclareLaunchArgument('num_walkers', default_value='0'),
        DeclareLaunchArgument('with_scenario', default_value='false'),
        DeclareLaunchArgument('with_yaml_scenario', default_value='false'),
    ]

    ego_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sim_bringup'), 'launch', 'ego_vehicle.launch.py')
        ),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'town': LaunchConfiguration('town'),
            'role_name': LaunchConfiguration('role_name'),
        }.items()
    )

    traffic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sim_traffic'), 'launch', 'seeded_traffic.launch.py')
        ),
        launch_arguments={
            'seed': LaunchConfiguration('seed'),
            'num_vehicles': LaunchConfiguration('num_vehicles'),
            'num_walkers': LaunchConfiguration('num_walkers'),
        }.items()
    )

    g29_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('g29_control'), 'launch', 'manual_recording.launch.py')
        ),
        launch_arguments={
            'role_name': LaunchConfiguration('role_name'),
        }.items()
    )

    scenario = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sim_scenarios'), 'launch', 'run_scenario.launch.py')
        ),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'role_name': LaunchConfiguration('role_name'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_scenario')),
    )

    yaml_scenario = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sim_scenarios_yaml'), 'launch', 'run_yaml_scenario.launch.py')
        ),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_yaml_scenario')),
    )

    return LaunchDescription(
        declarations + [ego_vehicle, traffic, g29_control, scenario, yaml_scenario])
