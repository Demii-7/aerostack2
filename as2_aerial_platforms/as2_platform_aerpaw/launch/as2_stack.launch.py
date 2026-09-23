"""AeroStack2 ROBOTICS SUBSYSTEM launch (AERPAW-owned lifecycle).

Brings up ONLY the AeroStack2 side of the integration, one full stack per UAV:
platform adapter node + state estimator + motion controller + behaviors. It starts
NO aerpawlib (AERPAW owns that). See Ownership note below and docs/ARCHITECTURE.md.

Stage 18 - configuration: the fleet (number of UAVs, ids, namespaces, IPC ports,
MAVLink endpoints, backend, TF frames, per-UAV params) is read from a declarative
YAML (``config/fleet.yaml`` by default, override with ``fleet_config:=``). Change the
number of UAVs / platform config by editing that file — NOT this source. Launch args
(``num_drones``, ``platform_backend``, ``vehicle_ids``) override the file for ad-hoc runs.

Ownership (Stage 2): AERPAW starts this subsystem + the aerpaw_as2_runner per UAV.
"""

__authors__ = 'AERPAW Bridge'
__copyright__ = 'Copyright (c) 2024 Universidad Politecnica de Madrid'
__license__ = 'BSD-3-Clause'

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# fleet_config.py is a sibling file in this launch dir (installed + in-repo).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fleet_config import load_fleet  # noqa: E402


def _as2_stack_nodes(ns, use_sim_time, frames):
    """AS2 per-drone robotics stack (estimator + controller + behaviors).

    Frame ids come from config (``frames``), not hardcoded suffixes.
    """
    state_estimator_pkg = get_package_share_directory('as2_state_estimator')
    motion_controller_pkg = get_package_share_directory('as2_motion_controller')
    motion_behaviors_pkg = get_package_share_directory('as2_behaviors_motion')

    f_base = f'{ns}/{frames["base"]}'
    f_earth = f'{ns}/{frames["earth"]}'
    f_odom = f'{ns}/{frames["odom"]}'

    def behavior(pkg, exe, name, plugin_dir, plugin):
        return Node(
            package=pkg, executable=exe, name=name, namespace=ns,
            output='screen', emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(motion_behaviors_pkg, plugin_dir, 'config',
                             'config_default.yaml'),
                {'plugin_name': plugin},
            ],
        )

    nodes = [
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node', name='state_estimator', namespace=ns,
            output='screen', emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time, 'base_frame': f_base,
                 'global_frame': f_earth, 'odom_frame': f_odom},
                os.path.join(state_estimator_pkg, 'plugins', 'raw_odometry',
                             'config', 'plugin_default.yaml'),
                {'plugin_name': 'raw_odometry'},
            ],
        ),
        Node(
            package='as2_motion_controller',
            executable='as2_motion_controller_node', name='motion_controller',
            namespace=ns, output='screen', emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(motion_controller_pkg, 'config',
                             'motion_controller_default.yaml'),
                {'plugin_name': 'pid_speed_controller'},
                {'plugin_available_modes_config_file': os.path.join(
                    motion_controller_pkg, 'plugins', 'pid_speed_controller',
                    'config', 'available_modes.yaml')},
                os.path.join(motion_controller_pkg, 'plugins', 'pid_speed_controller',
                             'config', 'controller_default.yaml'),
            ],
        ),
        Node(package='as2_behaviors_platform', executable='arm_behavior',
             name='arm_behavior', namespace=ns, output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='as2_behaviors_platform', executable='offboard_behavior',
             name='offboard_behavior', namespace=ns, output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': use_sim_time}]),
        behavior('as2_behaviors_motion', 'go_to_behavior_node', 'GoToBehavior',
                 'go_to_behavior', 'go_to_plugin_position'),
        behavior('as2_behaviors_motion', 'takeoff_behavior_node', 'TakeoffBehavior',
                 'takeoff_behavior', 'takeoff_plugin_platform'),
        behavior('as2_behaviors_motion', 'land_behavior_node', 'LandBehavior',
                 'land_behavior', 'land_plugin_platform'),
        behavior('as2_behaviors_motion', 'follow_path_behavior_node', 'FollowPathBehavior',
                 'follow_path_behavior', 'follow_path_plugin_position'),
        behavior('as2_behaviors_motion', 'follow_reference_behavior_node',
                 'FollowReferenceBehavior', 'follow_reference_behavior',
                 'follow_reference_plugin_position'),
    ]
    return nodes


def spawn_subsystem(context, *args, **kwargs):
    """Build the AS2 subsystem from the fleet config (no aerpawlib)."""
    platform_pkg = get_package_share_directory('as2_platform_aerpaw')
    default_control_modes = os.path.join(platform_pkg, 'config', 'control_modes.yaml')
    platform_params = os.path.join(platform_pkg, 'config', 'platform_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Resolve the fleet from YAML; launch args override the file for ad-hoc runs.
    fleet_path = LaunchConfiguration('fleet_config').perform(context).strip() or None
    nd = LaunchConfiguration('num_drones').perform(context).strip()
    num_override = int(nd) if nd else None
    fleet = load_fleet(fleet_path, num_drones=num_override)

    # Backend precedence (config-first): platform_backend arg > use_aerpaw arg (if
    # explicitly set) > fleet.yaml (per-vehicle, then global). use_aerpaw='' means
    # "let the fleet config decide".
    backend_arg = LaunchConfiguration('platform_backend').perform(context).strip()
    use_aerpaw = LaunchConfiguration('use_aerpaw').perform(context).strip().lower()
    arg_backend = ''
    if not backend_arg and use_aerpaw in ('true', '1', 'yes', 'on'):
        arg_backend = 'digital_twin'
    elif not backend_arg and use_aerpaw in ('false', '0', 'no', 'off'):
        arg_backend = 'sitl'
    forced_backend = backend_arg or arg_backend
    vids = LaunchConfiguration('vehicle_ids').perform(context).split()

    actions = []
    for i, veh in enumerate(fleet['vehicles']):
        ns = veh['namespace']
        backend = forced_backend or veh['backend']
        vehicle_id = vids[i] if i < len(vids) and vids[i] else veh['id']
        cm = veh['params'].get('control_modes_file', default_control_modes)

        node_params = {
            'use_sim_time': use_sim_time,
            'control_modes_file': cm,
            'ipc_cmd_port': veh['cmd_port'],
            'ipc_tel_port': veh['tel_port'],
            'platform_backend': backend,
            'vehicle_id': vehicle_id,
            'earth_frame_id': fleet['frames']['earth'],
            'map_frame_id': fleet['frames']['map'],
            'odom_frame_id': fleet['frames']['odom'],
            'base_frame_id': fleet['frames']['base'],
        }
        # Merge per-UAV params (takeoff_altitude, link_timeout, position_* ...) from config.
        for k in ('takeoff_altitude', 'link_timeout', 'position_update_min_distance',
                  'position_update_min_yaw', 'position_keepalive', 'cmd_freq', 'info_freq'):
            if k in veh['params']:
                node_params[k] = veh['params'][k]

        actions.append(Node(
            package='as2_platform_aerpaw', executable='as2_platform_aerpaw_node',
            name='platform', namespace=ns, output='screen', emulate_tty=True,
            parameters=[platform_params, node_params],
        ))
        actions.extend(_as2_stack_nodes(ns, use_sim_time, fleet['frames']))

    return actions


def generate_launch_description() -> LaunchDescription:
    """Entry point. AERPAW starts this to stand up the AS2 robotics subsystem."""
    actions = [
        DeclareLaunchArgument('fleet_config', default_value='',
                              description='Fleet YAML path (default: package config/fleet.yaml)'),
        DeclareLaunchArgument('num_drones', default_value='',
                              description='Override vehicle count from fleet_config (empty=use file)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('use_aerpaw', default_value='',
                              description='Override backend by env: true=digital_twin, false=sitl '
                                          '(empty=use fleet.yaml backend)'),
        DeclareLaunchArgument('platform_backend', default_value='',
                              description='Override backend: sitl|digital_twin|physical'),
        DeclareLaunchArgument('vehicle_ids', default_value='',
                              description='Override AERPAW vehicle names (space-separated)'),
    ]
    actions.append(OpaqueFunction(function=spawn_subsystem))
    return LaunchDescription(actions)
