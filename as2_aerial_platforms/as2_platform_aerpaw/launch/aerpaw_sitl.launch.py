"""Launch file for N AERPAW platform drones on SITL/AERPAW DT."""

__authors__ = 'AERPAW Bridge'
__copyright__ = 'Copyright (c) 2024 Universidad Politecnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_package_config_file():
    """Return the platform params config file."""
    package_folder = get_package_share_directory('as2_platform_aerpaw')
    return os.path.join(package_folder, 'config/platform_params.yaml')


def spawn_drones(context, *args, **kwargs):
    """Spawn N drone platform nodes + aerpawlib runners."""
    package_folder = get_package_share_directory('as2_platform_aerpaw')
    control_modes = os.path.join(package_folder, 'config/control_modes.yaml')
    runner_script = os.path.join(package_folder, 'aerpawlib_runner', 'aerpaw_as2_runner.py')

    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    conn_base = LaunchConfiguration('conn_base').perform(context)
    use_sim_time = str(LaunchConfiguration('use_sim_time').perform(
        context)).strip().lower() in ('1', 'true', 'yes', 'on')
    use_aerpaw = LaunchConfiguration('use_aerpaw').perform(context)

    conns = []
    for i in range(num_drones):
        conns.append(LaunchConfiguration(f'conn_drone{i}').perform(context))

    actions = []
    for i in range(num_drones):
        ns = f'drone{i}'
        cmd_port = str(15760 + i * 2)
        tel_port = str(15761 + i * 2)

        conn = conns[i] if conns[i] else f'udpin://127.0.0.1:{14550 + i * 10}'

        platform_node = Node(
            package='as2_platform_aerpaw',
            executable='as2_platform_aerpaw_node',
            name='platform',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'control_modes_file': control_modes,
                    'ipc_cmd_port': cmd_port,
                    'ipc_tel_port': tel_port,
                },
                get_package_config_file(),
            ]
        )
        actions.append(platform_node)

        runner_cmd = [
            'aerpawlib',
            '--api-version', 'v2',
            '--script', runner_script,
            '--conn', conn,
            '--vehicle', 'drone',
        ]
        if use_aerpaw != 'true':
            runner_cmd.append('--no-aerpaw-environment')
        runner_cmd += ['--cmd-port', cmd_port, '--tel-port', tel_port]

        actions.append(ExecuteProcess(
            cmd=runner_cmd,
            output='screen',
            additional_env={'PYTHONPATH': ''},
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    """Entry point for launch file."""
    ld = LaunchDescription([
        DeclareLaunchArgument('num_drones',
                              default_value='3',
                              description='Number of drones'),
        DeclareLaunchArgument('conn_base',
                              default_value='udpin://127.0.0.1:14550',
                              description='Base MAVLink connection string'),
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('use_aerpaw',
                              default_value='false',
                              description='Use AERPAW environment'),
    ])

    for i in range(10):
        ld.add_action(DeclareLaunchArgument(
            f'conn_drone{i}',
            default_value='',
            description=f'MAVLink connection for drone{i} (overrides conn_base)'))

    ld.add_action(OpaqueFunction(function=spawn_drones))
    return ld