"""Config C: Launch 3 drones on the AERPAW Digital Twin / SITL.

Uses the as2_platform_aerpaw aerpaw_sitl.launch.py internally,
plus the full AS2 stack (state estimator, motion controller, behaviors)
for each drone.

For real AERPAW Digital Twin:
  ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \\
      use_aerpaw:=true \\
      conn_drone0:=udpin://10.14.X.X:14550 \\
      conn_drone1:=udpin://10.14.X.Y:14550 \\
      conn_drone2:=udpin://10.14.X.Z:14550

For SITL validation:
  ros2 launch as2_experiment_aerpaw_multiuav aerpaw_digital_twin.launch.py \\
      use_aerpaw:=false
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
from launch_ros.actions import Node


def _as2_stack_nodes(ns, use_sim_time):
    """Return list of nodes for the full AS2 drone stack in namespace ``ns``."""
    state_estimator_pkg = get_package_share_directory('as2_state_estimator')
    motion_controller_pkg = get_package_share_directory('as2_motion_controller')
    motion_behaviors_pkg = get_package_share_directory('as2_behaviors_motion')

    nodes = []

    # --- State estimator (raw_odometry plugin: reads sensor_measurements/odom) ---
    nodes.append(Node(
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
            os.path.join(state_estimator_pkg, 'plugins', 'raw_odometry', 'config', 'plugin_default.yaml'),
            {'plugin_name': 'raw_odometry'},
        ],
    ))

    # --- Motion controller (PID speed) ---
    nodes.append(Node(
        package='as2_motion_controller',
        executable='as2_motion_controller_node',
        name='motion_controller',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(motion_controller_pkg, 'config', 'motion_controller_default.yaml'),
            {'plugin_name': 'pid_speed_controller'},
            {'plugin_available_modes_config_file': os.path.join(
                motion_controller_pkg, 'plugins', 'pid_speed_controller', 'config',
                'available_modes.yaml')},
            os.path.join(
                motion_controller_pkg, 'plugins', 'pid_speed_controller', 'config',
                'controller_default.yaml'),
        ],
    ))

    # --- Platform behaviors (arm, offboard) ---
    nodes.append(Node(
        package='as2_behaviors_platform',
        executable='arm_behavior',
        name='arm_behavior',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    ))
    nodes.append(Node(
        package='as2_behaviors_platform',
        executable='offboard_behavior',
        name='offboard_behavior',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    # --- Motion behaviors (go_to, takeoff, land) ---
    nodes.append(Node(
        package='as2_behaviors_motion',
        executable='go_to_behavior_node',
        name='GoToBehavior',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(motion_behaviors_pkg, 'go_to_behavior', 'config', 'config_default.yaml'),
            {'plugin_name': 'go_to_plugin_position'},
        ],
    ))
    nodes.append(Node(
        package='as2_behaviors_motion',
        executable='takeoff_behavior_node',
        name='TakeoffBehavior',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(motion_behaviors_pkg, 'takeoff_behavior', 'config', 'config_default.yaml'),
            {'plugin_name': 'takeoff_plugin_platform'},
        ],
    ))
    nodes.append(Node(
        package='as2_behaviors_motion',
        executable='land_behavior_node',
        name='LandBehavior',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(motion_behaviors_pkg, 'land_behavior', 'config', 'config_default.yaml'),
            {'plugin_name': 'land_plugin_platform'},
        ],
    ))

    return nodes


def get_all_actions(context, *args, **kwargs):
    """Include aerpaw_sitl.launch.py + AS2 stack + mission script."""
    num = int(LaunchConfiguration('num_drones').perform(context))
    sim_time = LaunchConfiguration('use_sim_time').perform(context)
    mission_pkg = get_package_share_directory('as2_experiment_aerpaw_multiuav')
    platform_pkg = get_package_share_directory('as2_platform_aerpaw')

    actions = []

    # Include aerpaw_sitl.launch.py from the platform package, forwarding all args
    include_args = {
        'num_drones': LaunchConfiguration('num_drones'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'use_aerpaw': LaunchConfiguration('use_aerpaw'),
    }
    for i in range(num):
        include_args[f'conn_drone{i}'] = LaunchConfiguration(f'conn_drone{i}')

    sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(platform_pkg, 'launch/aerpaw_sitl.launch.py')),
        launch_arguments=include_args.items(),
    )
    actions.append(sitl_launch)

    # AS2 stack for each drone
    for i in range(num):
        ns = f'drone{i}'
        actions.extend(_as2_stack_nodes(ns, sim_time))

    # Mission script
    actions.append(ExecuteProcess(
        cmd=['python3', os.path.join(mission_pkg, 'missions/triangle_formation.py')],
        output='screen',
    ))

    return actions


def generate_launch_description() -> LaunchDescription:
    actions = [
        DeclareLaunchArgument('num_drones', default_value='3'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_aerpaw', default_value='false',
                              description='true for AERPAW DT, false for SITL'),
    ]

    # Per-drone connection strings (up to 10 drones)
    for i in range(10):
        actions.append(DeclareLaunchArgument(
            f'conn_drone{i}',
            default_value='',
            description=f'MAVLink connection string for drone{i}'))

    actions.append(OpaqueFunction(function=get_all_actions))
    return LaunchDescription(actions)
