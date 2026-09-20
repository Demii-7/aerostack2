"""Launch file for a single AERPAW platform drone."""

__authors__ = 'AERPAW Bridge'
__copyright__ = 'Copyright (c) 2024 Universidad Politecnica de Madrid'
__license__ = 'BSD-3-Clause'

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IfCondition, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_package_config_file():
    """Return the platform params config file."""
    package_folder = get_package_share_directory('as2_platform_aerpaw')
    return os.path.join(package_folder, 'config/platform_params.yaml')


def generate_launch_description() -> LaunchDescription:
    """Entry point for launch file."""
    package_folder = get_package_share_directory('as2_platform_aerpaw')
    control_modes = os.path.join(package_folder, 'config/control_modes.yaml')
    runner_script = os.path.join(package_folder, 'aerpawlib_runner', 'aerpaw_as2_runner.py')

    return LaunchDescription([
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'),
        DeclareLaunchArgument('use_sim_time',
                              description='Use simulation clock if true',
                              default_value='false'),
        DeclareLaunchArgument('namespace',
                              description='Drone namespace',
                              default_value='drone0'),
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('conn',
                              default_value='udpin://127.0.0.1:14550',
                              description='MAVLink connection string'),
        DeclareLaunchArgument('cmd_port',
                              default_value='15760',
                              description='IPC command UDP port'),
        DeclareLaunchArgument('tel_port',
                              default_value='15761',
                              description='IPC telemetry UDP port'),
        DeclareLaunchArgument('use_aerpaw',
                              default_value='false',
                              description='Use AERPAW environment'),
        Node(
            package='as2_platform_aerpaw',
            executable='as2_platform_aerpaw_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'control_modes_file': LaunchConfiguration('control_modes_file'),
                    'ipc_cmd_port': LaunchConfiguration('cmd_port'),
                    'ipc_tel_port': LaunchConfiguration('tel_port'),
                },
                get_package_config_file(),
            ]
        ),
        LogInfo(msg=['Launching aerpaw_as2_runner with connection: ',
                      LaunchConfiguration('conn')]),
        ExecuteProcess(
            cmd=[
                'aerpawlib',
                '--api-version', 'v2',
                '--script', runner_script,
                '--conn', LaunchConfiguration('conn'),
                '--vehicle', 'drone',
                '--no-aerpaw-environment',
                '--cmd-port', LaunchConfiguration('cmd_port'),
                '--tel-port', LaunchConfiguration('tel_port'),
            ],
            condition=UnlessCondition(LaunchConfiguration('use_aerpaw')),
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'aerpawlib',
                '--api-version', 'v2',
                '--script', runner_script,
                '--conn', LaunchConfiguration('conn'),
                '--vehicle', 'drone',
                '--cmd-port', LaunchConfiguration('cmd_port'),
                '--tel-port', LaunchConfiguration('tel_port'),
            ],
            condition=IfCondition(LaunchConfiguration('use_aerpaw')),
            output='screen',
        ),
    ])
