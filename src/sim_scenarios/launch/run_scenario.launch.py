"""
Run a custom OpenSCENARIO (.xosc) scenario against a CARLA world that is
already running with the bridge as ticking master and the ego vehicle
spawned (see sim_bringup's ego_vehicle.launch.py).

This package does not implement scenario logic itself -- it orchestrates
the external `scenario_runner` CARLA project (not vendored here) through
carla_ros_scenario_runner, the same way carla_ad_demo does upstream.

Setup:
  git clone https://github.com/carla-simulator/scenario_runner.git
  export SCENARIO_RUNNER_PATH=/path/to/scenario_runner

Usage:
  ros2 launch sim_scenarios run_scenario.launch.py
  ros2 launch sim_scenarios run_scenario.launch.py \
      scenario_file:=/abs/path/to/my_scenario.xosc scenario_name:=MyScenario

To write your own scenario, copy scenarios/FollowLeadingVehicle.xosc (the
stock CARLA example, vendored here for reference) and edit it, or author one
from scratch per the OpenSCENARIO spec that scenario_runner consumes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    scenario_name = LaunchConfiguration('scenario_name').perform(context)
    scenario_file = LaunchConfiguration('scenario_file').perform(context)
    auto_execute = LaunchConfiguration('auto_execute').perform(context).lower() == 'true'

    scenario_list_yaml = (
        "{scenarios: [{name: '%s', scenario_file: '%s'}]}"
        % (scenario_name, scenario_file)
    )
    execute_request_yaml = (
        "{scenario: {name: '%s', scenario_file: '%s'}}"
        % (scenario_name, scenario_file)
    )

    actions = [
        # Advertise the scenario on /carla/available_scenarios, the same
        # topic the RVIZ Carla Plugin reads from (not required here, but
        # kept for compatibility if you later add that plugin).
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/carla/available_scenarios',
                'carla_ros_scenario_runner_types/msg/CarlaScenarioList',
                scenario_list_yaml],
            name='publish_available_scenarios',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('carla_ros_scenario_runner'),
                             'carla_ros_scenario_runner.launch.py')
            ),
            launch_arguments={
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'role_name': LaunchConfiguration('role_name'),
                'scenario_runner_path': LaunchConfiguration('scenario_runner_path'),
                'wait_for_ego': 'True',
            }.items()
        ),
    ]

    if auto_execute:
        # Give carla_ros_scenario_runner time to come up and find the ego
        # before triggering execution. Equivalent to pressing "Execute" in
        # the RVIZ plugin.
        actions.append(TimerAction(
            period=5.0,
            actions=[ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                    '/scenario_runner/execute_scenario',
                    'carla_ros_scenario_runner_types/srv/ExecuteScenario',
                    execute_request_yaml],
                name='trigger_scenario_execution',
            )],
        ))

    return actions


def generate_launch_description():
    default_scenario_file = os.path.join(
        get_package_share_directory('sim_scenarios'),
        'scenarios', 'FollowLeadingVehicle.xosc')

    declarations = [
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('role_name', default_value='ego_vehicle'),
        DeclareLaunchArgument(
            'scenario_runner_path',
            default_value=os.environ.get('SCENARIO_RUNNER_PATH', ''),
            description='Path to a scenario_runner checkout (SCENARIO_RUNNER_PATH env by default).'),
        DeclareLaunchArgument('scenario_name', default_value='FollowLeadingVehicle'),
        DeclareLaunchArgument('scenario_file', default_value=default_scenario_file),
        DeclareLaunchArgument(
            'auto_execute', default_value='true',
            description='Automatically trigger the scenario a few seconds after start, '
                        'instead of waiting for a manual service call.'),
    ]

    return LaunchDescription(declarations + [OpaqueFunction(function=launch_setup)])
