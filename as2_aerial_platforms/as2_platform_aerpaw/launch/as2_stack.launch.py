"""AeroStack2 ROBOTICS SUBSYSTEM launch (AERPAW-owned lifecycle).

This launch brings up ONLY the AeroStack2 side of the integration for N drones:

    as2_platform_aerpaw platform node   (the AERPAW<->AS2 adapter)
      + as2_state_estimator              (per drone)
      + as2_motion_controller            (per drone)
      + as2_behaviors_platform           (arm / offboard, per drone)
      + as2_behaviors_motion             (go_to / takeoff / land, per drone)

It deliberately starts NOTHING on the AERPAW side: no ``aerpawlib``, no vehicle
connection handling, no mission.  AeroStack2 is a robotics *subsystem*; AERPAW is
the top-level experiment platform and is responsible for starting/stopping this
subsystem and for launching the ``aerpaw_as2_runner`` that provides the MAVLink
endpoint this platform node talks to over its UDP IPC socket.

Ownership (Stage 2 - AERPAW is top-level):

    AERPAW experiment  --starts-->  aerpaw_as2_runner.py  (per drone, MAVLink)
                   \\--starts-->  ros2 launch as2_platform_aerpaw as2_stack.launch.py
                                   (this file: the AS2 robotics subsystem)

AERPAW is reached over the platform's localhost UDP IPC (see ipc_bridge); the
MAVLink ``--conn`` is owned by the runner, NOT by this launch file.

Example (invoked by the AERPAW experiment / deploy script, not by hand):

    ros2 launch as2_platform_aerpaw as2_stack.launch.py num_drones:=3 use_aerpaw:=true
"""

__authors__ = 'AERPAW Bridge'
__copyright__ = 'Copyright (c) 2024 Universidad Politecnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as2_stack_nodes(ns, use_sim_time):
    """AS2 per-drone robotics stack (estimator + controller + behaviors).

    No platform node and no aerpawlib here; those are added by the caller /
    owned by AERPAW respectively.
    """
    state_estimator_pkg = get_package_share_directory('as2_state_estimator')
    motion_controller_pkg = get_package_share_directory('as2_motion_controller')
    motion_behaviors_pkg = get_package_share_directory('as2_behaviors_motion')

    nodes = [
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node',
            name='state_estimator',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time,
                 'base_frame': f'{ns}/base_link',
                 'global_frame': f'{ns}/earth',
                 'odom_frame': f'{ns}/odom'},
                os.path.join(
                    state_estimator_pkg, 'plugins', 'raw_odometry', 'config',
                    'plugin_default.yaml'),
                {'plugin_name': 'raw_odometry'},
            ],
        ),
        Node(
            package='as2_motion_controller',
            executable='as2_motion_controller_node',
            name='motion_controller',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    motion_controller_pkg, 'config', 'motion_controller_default.yaml'),
                {'plugin_name': 'pid_speed_controller'},
                {'plugin_available_modes_config_file': os.path.join(
                    motion_controller_pkg, 'plugins', 'pid_speed_controller', 'config',
                    'available_modes.yaml')},
                os.path.join(
                    motion_controller_pkg, 'plugins', 'pid_speed_controller', 'config',
                    'controller_default.yaml'),
            ],
        ),
        Node(
            package='as2_behaviors_platform',
            executable='arm_behavior',
            name='arm_behavior',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='as2_behaviors_platform',
            executable='offboard_behavior',
            name='offboard_behavior',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='as2_behaviors_motion',
            executable='go_to_behavior_node',
            name='GoToBehavior',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    motion_behaviors_pkg, 'go_to_behavior', 'config', 'config_default.yaml'),
                {'plugin_name': 'go_to_plugin_position'},
            ],
        ),
        Node(
            package='as2_behaviors_motion',
            executable='takeoff_behavior_node',
            name='TakeoffBehavior',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    motion_behaviors_pkg, 'takeoff_behavior', 'config',
                    'config_default.yaml'),
                {'plugin_name': 'takeoff_plugin_platform'},
            ],
        ),
        Node(
            package='as2_behaviors_motion',
            executable='land_behavior_node',
            name='LandBehavior',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    motion_behaviors_pkg, 'land_behavior', 'config', 'config_default.yaml'),
                {'plugin_name': 'land_plugin_platform'},
            ],
        ),
        # Reusable AS2 follow behaviors (position plugins -> need only the
        # POSITION platform mode AERPAW supports). No AERPAW-specific behavior code.
        # The *_plugin_trajectory variants need a TRAJECTORY platform mode, which
        # AERPAW cannot execute on the testbed (offboard is filter-blocked) - see
        # docs/AS2_CAPABILITIES.md.
        Node(
            package='as2_behaviors_motion',
            executable='follow_path_behavior_node',
            name='FollowPathBehavior',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    motion_behaviors_pkg, 'follow_path_behavior', 'config',
                    'config_default.yaml'),
                {'plugin_name': 'follow_path_plugin_position'},
            ],
        ),
        Node(
            package='as2_behaviors_motion',
            executable='follow_reference_behavior_node',
            name='FollowReferenceBehavior',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    motion_behaviors_pkg, 'follow_reference_behavior', 'config',
                    'config_default.yaml'),
                {'plugin_name': 'follow_reference_plugin_position'},
            ],
        ),
    ]
    return nodes


def spawn_subsystem(context, *args, **kwargs):
    """Build the per-drone AS2 platform node + robotics stack (no aerpawlib)."""
    platform_pkg = get_package_share_directory('as2_platform_aerpaw')
    control_modes = os.path.join(platform_pkg, 'config', 'control_modes.yaml')
    platform_params = os.path.join(platform_pkg, 'config', 'platform_params.yaml')

    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_aerpaw = LaunchConfiguration('use_aerpaw').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    # Backend is AERPAW's environment label. DT and physical share one capability
    # class (the AERPAW environment); switching between them is pure config, no code
    # change. Default follows use_aerpaw (DT when on AERPAW), overridable via the
    # platform_backend arg (e.g. physical).
    backend = LaunchConfiguration('platform_backend').perform(context).strip()
    if not backend:
        backend = 'digital_twin' if use_aerpaw else 'sitl'
    # Optional explicit AERPAW vehicle names, one per drone (space-separated).
    vehicle_ids = LaunchConfiguration('vehicle_ids').perform(context).split()

    actions = []
    for i in range(num_drones):
        # Mirror of vehicle_identity.hpp (single source of truth in the adapter):
        ns = f'drone{i}'
        cmd_port = 15760 + i * 2
        tel_port = 15761 + i * 2
        # Unique id per AERPAW UAV: explicit name if given, else the namespace.
        vehicle_id = vehicle_ids[i] if i < len(vehicle_ids) and vehicle_ids[i] else ns

        # AS2 platform node = the AERPAW<->AS2 adapter.  It connects to the
        # aerpawlib runner (started by AERPAW) only through this localhost UDP
        # IPC pair; it holds no MAVLink/AERPAW connection of its own.
        actions.append(Node(
            package='as2_platform_aerpaw',
            executable='as2_platform_aerpaw_node',
            name='platform',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[
                platform_params,
                {
                    'use_sim_time': use_sim_time,
                    'control_modes_file': control_modes,
                    'ipc_cmd_port': cmd_port,
                    'ipc_tel_port': tel_port,
                    'platform_backend': backend,
                    'vehicle_id': vehicle_id,
                },
            ],
        ))
        actions.extend(_as2_stack_nodes(ns, use_sim_time))

    return actions


def generate_launch_description() -> LaunchDescription:
    """Entry point. AERPAW starts this to stand up the AS2 robotics subsystem."""
    actions = [
        DeclareLaunchArgument('num_drones', default_value='1',
                              description='Number of AS2 drone subsystems'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('use_aerpaw', default_value='false',
                              description='true = AERPAW environment backend, false = SITL'),
        DeclareLaunchArgument('platform_backend', default_value='',
                              description='Override AERPAW env label: sitl|digital_twin|physical '
                                          '(DT and physical are capability-identical)'),
        DeclareLaunchArgument('vehicle_ids', default_value='',
                              description='Optional AERPAW vehicle names, one per drone'),
    ]
    actions.append(OpaqueFunction(function=spawn_subsystem))
    return LaunchDescription(actions)
