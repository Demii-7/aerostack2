"""Config B: Launch 3 drones on Gazebo with full AS2 stack.

Requires a Gazebo simulation config YAML that defines 3 drone models.
Save it somewhere and pass it via ``simulation_config_file``, e.g.:

  ros2 launch as2_experiment_aerpaw_multiuav sim_gazebo.launch.py \\
      simulation_config_file:=/path/to/three_drone_world.yaml
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

    # --- State estimator (ground_truth: Gazebo publishes ground_truth/*) ---
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
            os.path.join(state_estimator_pkg, 'ground_truth', 'config', 'plugin_default.yaml'),
            {'plugin_name': 'ground_truth'},
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
            {'plugin_name': 'pid_speed'},
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
    """Build all launch actions for N Gazebo drones."""
    config_file = LaunchConfiguration('simulation_config_file').perform(context)
    num = int(LaunchConfiguration('num_drones').perform(context))
    sim_time = LaunchConfiguration('use_sim_time').perform(context)

    gazebo_pkg = get_package_share_directory('as2_gazebo_assets')
    platform_pkg = get_package_share_directory('as2_platform_gazebo')
    mission_pkg = get_package_share_directory('as2_experiment_aerpaw_multiuav')

    actions = []

    # 1. Gazebo world + models + world/object bridges
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch/launch_simulation.py')),
        launch_arguments={'simulation_config_file': config_file}.items(),
    )
    actions.append(sim_launch)

    # 2. Per-drone: bridges + platform node + AS2 stack
    for i in range(num):
        ns = f'drone{i}'

        drone_bridges = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch/drone_bridges.py')),
            launch_arguments={
                'simulation_config_file': config_file,
                'namespace': ns,
            }.items(),
        )
        actions.append(drone_bridges)

        platform_control = os.path.join(platform_pkg, 'config/control_modes.yaml')
        platform_config = os.path.join(platform_pkg, 'config/platform_config_file.yaml')

        actions.append(Node(
            package='as2_platform_gazebo',
            executable='as2_platform_gazebo_node',
            name='platform',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': sim_time,
                'control_modes_file': platform_control,
                'cmd_vel_topic': f'/gz/{ns}/cmd_vel',
                'arm_topic': f'/gz/{ns}/arm',
                'acro_topic': f'/gz/{ns}/acro',
            },
                platform_config,
            ],
        ))

        # Full AS2 stack (state estimator, motion controller, behaviors)
        actions.extend(_as2_stack_nodes(ns, sim_time))

    # 3. Mission script
    actions.append(ExecuteProcess(
        cmd=['python3', os.path.join(mission_pkg, 'missions/triangle_formation.py')],
        output='screen',
    ))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('simulation_config_file',
                              description='YAML world file with 3 drones defined'),
        DeclareLaunchArgument('num_drones', default_value='3',
                              description='Number of drones'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=get_all_actions),
    ])
