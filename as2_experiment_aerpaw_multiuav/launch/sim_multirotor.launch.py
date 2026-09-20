"""Config A: Launch N drones on the AS2 Multirotor Simulator with full stack.

  ros2 launch as2_experiment_aerpaw_multiuav sim_multirotor.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as2_stack_nodes(ns, use_sim_time, params=None):
    """Return list of nodes for the full AS2 drone stack in namespace ``ns``."""
    state_estimator_pkg = get_package_share_directory('as2_state_estimator')
    motion_controller_pkg = get_package_share_directory('as2_motion_controller')

    state_estimator_config = os.path.join(
        state_estimator_pkg, 'raw_odometry', 'config', 'plugin_default.yaml')
    motion_controller_config = os.path.join(motion_controller_pkg, 'config',
                                            'motion_controller_default.yaml')

    nodes = []

    # --- State estimator (raw_odometry plugin) ---
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
            state_estimator_config,
            {'plugin_name': 'raw_odometry'},
        ],
    ))

    # --- Motion controller ---
    nodes.append(Node(
        package='as2_motion_controller',
        executable='as2_motion_controller_node',
        name='motion_controller',
        namespace=ns,
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            motion_controller_config,
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
    motion_behaviors_pkg = get_package_share_directory('as2_behaviors_motion')
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


def get_nodes(context, *args, **kwargs):
    """Spawn N multirotor simulator platform nodes with the full AS2 stack."""
    num = int(LaunchConfiguration('num_drones').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    sim_pkg = get_package_share_directory('as2_platform_multirotor_simulator')
    sim_config = os.path.join(sim_pkg, 'config/platform_config_file.yaml')
    sim_control = os.path.join(sim_pkg, 'config/control_modes.yaml')
    uav_config = os.path.join(sim_pkg, 'config/uav_config.yaml')
    world_config = os.path.join(sim_pkg, 'config/world_config.yaml')

    nodes = []
    for i in range(num):
        ns = f'drone{i}'
        # Multirotor simulator platform node
        nodes.append(Node(
            package='as2_platform_multirotor_simulator',
            executable='as2_platform_multirotor_simulator_node',
            name='platform',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'control_modes_file': sim_control,
                'vehicle_initial_pose.x': 0.0,
                'vehicle_initial_pose.y': float(i) * 10.0,
                'vehicle_initial_pose.z': 0.0,
                'vehicle_initial_pose.yaw': 0.0,
            },
                sim_config,
                uav_config,
                world_config,
            ],
        ))
        # Full AS2 stack (state estimator, motion controller, behaviors)
        nodes.extend(_as2_stack_nodes(ns, use_sim_time))

    return nodes


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('as2_experiment_aerpaw_multiuav')
    mission_script = os.path.join(pkg, 'missions', 'triangle_formation.py')

    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='3',
                              description='Number of drones'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        OpaqueFunction(function=get_nodes),
        ExecuteProcess(
            cmd=['python3', mission_script],
            output='screen',
        ),
    ])
