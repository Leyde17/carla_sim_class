"""
Start the CARLA ROS bridge and spawn a single ego vehicle equipped with
ONLY a lidar (plus the pseudo-sensors needed for odometry/TF/control), using
config/objects_lidar_only.json.

  ros2 launch sim_bringup ego_vehicle.launch.py
  ros2 launch sim_bringup ego_vehicle.launch.py town:=Town05 host:=localhost
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_objects_file = os.path.join(
        get_package_share_directory('sim_bringup'), 'config', 'objects_lidar_only.json')

    declarations = [
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('town', default_value='Town01'),
        DeclareLaunchArgument('timeout', default_value='10'),
        DeclareLaunchArgument('role_name', default_value='ego_vehicle'),
        DeclareLaunchArgument('synchronous_mode', default_value='True'),
        DeclareLaunchArgument(
            'synchronous_mode_wait_for_vehicle_control_command', default_value='True'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05'),
        DeclareLaunchArgument('objects_definition_file', default_value=default_objects_file),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('carla_ros_bridge'),
                         'carla_ros_bridge.launch.py')
        ),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'town': LaunchConfiguration('town'),
            'timeout': LaunchConfiguration('timeout'),
            'synchronous_mode': LaunchConfiguration('synchronous_mode'),
            'synchronous_mode_wait_for_vehicle_control_command':
                LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
            'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds'),
        }.items()
    )

    spawn_ego = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('carla_spawn_objects'),
                         'carla_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': LaunchConfiguration('objects_definition_file'),
            'role_name': LaunchConfiguration('role_name'),
        }.items()
    )

    return LaunchDescription(declarations + [bridge, spawn_ego])
