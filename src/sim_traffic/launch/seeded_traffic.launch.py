"""
Launch the seeded dynamic traffic generator.

Parameters come from a YAML file, by default
<share>/sim_traffic/config/seeded_traffic.yaml.

  ros2 launch sim_traffic seeded_traffic.launch.py
  ros2 launch sim_traffic seeded_traffic.launch.py seed:=4321
  ros2 launch sim_traffic seeded_traffic.launch.py \
      params_file:=/abs/path/scenario_017.yaml

Launch the ros-bridge FIRST (it is the ticking master and spawns the ego):

  ros2 launch carla_ros_bridge carla_ros_bridge.launch.py \
      synchronous_mode:=True \
      synchronous_mode_wait_for_vehicle_control_command:=True \
      fixed_delta_seconds:=0.05 passive:=False town:=Town05
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'sim_traffic'
NODE_NAME = 'seeded_traffic'

# Launch args that may override a value from the YAML. Empty string means
# "leave the YAML value alone" -- this is what lets a single config file be
# reused across a seed sweep without editing it.
OVERRIDABLE = {
    'seed': int,
    'num_vehicles': int,
    'num_walkers': int,
}


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(params_file):
        raise RuntimeError('params_file not found: %s' % params_file)

    parameters = [params_file]

    overrides = {}
    for name, caster in OVERRIDABLE.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw != '':
            overrides[name] = caster(raw)
    if overrides:
        parameters.append(overrides)

    # Wall-clock timers only; sim time here can deadlock against the bridge.
    parameters.append({'use_sim_time': False})

    return [Node(
        package=PACKAGE,
        executable='seeded_traffic',   # console_scripts name, not the filename
        name=NODE_NAME,                # must match the top key in the YAML
        output='screen',
        emulate_tty=True,
        parameters=parameters,
    )]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'seeded_traffic.yaml')

    declarations = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the parameter YAML.'),
    ]
    declarations += [
        DeclareLaunchArgument(
            name, default_value='',
            description='Override %s from the YAML. Empty = use YAML value.'
                        % name)
        for name in OVERRIDABLE
    ]

    return LaunchDescription(declarations + [OpaqueFunction(function=launch_setup)])
