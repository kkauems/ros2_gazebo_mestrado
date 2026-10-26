import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


# Só os servidores necessários para NavigateToPose e FollowWaypoints. O
# navigation_launch.py do nav2_bringup (Jazzy) também sobe docking_server,
# que falha sem configuração e derruba o lifecycle_manager.
LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    webots_share = get_package_share_directory('agrobot_webots')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Com SLAM, o planejamento global vai para map: bt_navigator, costmap
    # global e behavior_server. O controller_server e o costmap local
    # continuam em odom (REP-105), senão o controlador pularia a cada
    # correção do SLAM.
    global_frame = LaunchConfiguration('global_frame')
    params_file = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={
            'bt_navigator.ros__parameters.global_frame': global_frame,
            'global_costmap.global_costmap.ros__parameters.global_frame':
                global_frame,
            'behavior_server.ros__parameters.global_frame': global_frame,
        },
        convert_types=True,
    )
    common = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                webots_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('global_frame', default_value='odom'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=common,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=common,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=common,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=common,
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=common,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=common,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': LIFECYCLE_NODES,
            }],
        ),
    ])
