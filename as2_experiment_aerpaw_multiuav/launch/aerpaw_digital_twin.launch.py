"""Config C: run a multi-UAV AERPAW SITL / Digital-Twin experiment (dev harness).

Stage-8 reuse: the ENTIRE AeroStack2 side (platform adapter + state estimator +
motion controller + the standard reusable behaviors: go_to / takeoff / land /
follow_path / follow_reference) is launched by REUSING the single subsystem
definition `as2_platform_aerpaw/launch/as2_stack.launch.py`.  Nothing here
re-implements a behavior or the AS2 stack.

This launch then only adds the AERPAW-side pieces the harness needs for a one-shot
local run: the aerpawlib runners (per drone) and the mission script.

NOTE (Stage-2 ownership): here *AeroStack2* spawns the *AERPAW* runners, so this is
a DEVELOPER / local-SITL VALIDATION HARNESS ONLY.  Production is AERPAW-first:
`deploy/run_aerpaw_experiment.sh` starts `as2_stack.launch.py` + runners + mission.
See docs/ARCHITECTURE.md §6.

For real AERPAW Digital Twin:
  ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \\
      use_aerpaw:=true \\
      conn_drone0:=udpin://10.14.X.X:14550 \\
      conn_drone1:=udpin://10.14.X.Y:14550 \\
      conn_drone2:=udpin://10.14.X.Z:14550

For SITL validation:
  ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py use_aerpaw:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def get_all_actions(context, *args, **kwargs):
    """Reuse as2_stack.launch.py (AS2 side) + start AERPAW runners + mission."""
    num = int(LaunchConfiguration('num_drones').perform(context))
    use_aerpaw = LaunchConfiguration('use_aerpaw').perform(context)

    experiment_pkg = get_package_share_directory('as2_experiment_aerpaw_multiuav')
    platform_pkg = get_package_share_directory('as2_platform_aerpaw')
    runner_script = os.path.join(
        platform_pkg, 'aerpawlib_runner', 'aerpaw_as2_runner.py')

    actions = []

    # --- 1. The whole AeroStack2 robotics subsystem (reused, not re-implemented) ---
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(platform_pkg, 'launch', 'as2_stack.launch.py')),
        launch_arguments={
            'num_drones': LaunchConfiguration('num_drones'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_aerpaw': LaunchConfiguration('use_aerpaw'),
            'platform_backend': LaunchConfiguration('platform_backend'),
        }.items(),
    ))

    # --- 2. AERPAW-side aerpawlib runners (one per drone; dev-harness inversion) ---
    for i in range(num):
        conn = LaunchConfiguration(f'conn_drone{i}').perform(context)
        if not conn:
            conn = f'udpin://127.0.0.1:{14550 + i * 10}'
        cmd_port = 15760 + i * 2
        tel_port = 15761 + i * 2
        runner_cmd = [
            'aerpawlib', '--api-version', 'v2',
            '--script', runner_script,
            '--conn', conn,
            '--vehicle', 'drone',
        ]
        if use_aerpaw != 'true':
            runner_cmd.append('--no-aerpaw-environment')
        runner_cmd += [
            '--cmd-port', str(cmd_port), '--tel-port', str(tel_port),
            '--vehicle-id', f'drone{i}']
        actions.append(ExecuteProcess(cmd=runner_cmd, output='screen'))

    # --- 3. Researcher mission (standard AS2 DroneInterface, unchanged) ---
    actions.append(ExecuteProcess(
        cmd=['python3', os.path.join(experiment_pkg, 'missions', 'triangle_formation.py')],
        output='screen',
    ))

    return actions


def generate_launch_description() -> LaunchDescription:
    actions = [
        DeclareLaunchArgument('num_drones', default_value='3'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_aerpaw', default_value='false',
                              description='true for AERPAW DT, false for SITL'),
        DeclareLaunchArgument('platform_backend', default_value='',
                              description='sitl|digital_twin|physical (empty=derive from use_aerpaw)'),
    ]

    # Per-drone connection strings (up to 10 drones)
    for i in range(10):
        actions.append(DeclareLaunchArgument(
            f'conn_drone{i}',
            default_value='',
            description=f'MAVLink connection string for drone{i}'))

    actions.append(OpaqueFunction(function=get_all_actions))
    return LaunchDescription(actions)
