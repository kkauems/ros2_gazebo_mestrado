import os
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


ODOM_FROM_DRIVER = '<publishOdom>true</publishOdom>'
ODOM_FROM_EKF = '<publishOdom>false</publishOdom>'


def driver_urdf(robot_description, ekf):
    """Caminho do URDF para o driver; com ekf, o driver não publica /odom."""
    if not ekf:
        return robot_description
    with open(robot_description) as urdf_file:
        content = urdf_file.read()
    if ODOM_FROM_DRIVER not in content:
        raise RuntimeError(
            f'{ODOM_FROM_DRIVER} não encontrado em {robot_description}')
    path = os.path.join(tempfile.gettempdir(), 'agrobot_webots_ekf.urdf')
    with open(path, 'w') as urdf_file:
        urdf_file.write(content.replace(ODOM_FROM_DRIVER, ODOM_FROM_EKF))
    return path


def generate_launch_description():
    webots_share = get_package_share_directory('agrobot_webots')
    world_path = PathJoinSubstitution([
        webots_share,
        'worlds',
        'obstacle_arena.wbt',
    ])

    webots = WebotsLauncher(
        world=world_path,
        output='screen',
    )

    description_share = get_package_share_directory('agrobot_description')
    robot_description = os.path.join(
        description_share,
        'urdf',
        'agrobot.urdf',
    )
    with open(robot_description) as urdf_file:
        robot_description_content = urdf_file.read()

    def start_driver(context):
        ekf = LaunchConfiguration('ekf').perform(context).lower() == 'true'
        return [WebotsController(
            robot_name='agrobot',
            parameters=[
                {'robot_description': driver_urdf(robot_description, ekf)},
                {'use_sim_time': True},
            ],
            output='screen',
        )]

    agrobot_driver = OpaqueFunction(function=start_driver)

    # Rodas + IMU -> /odom e TF odom -> base_link (no lugar do driver).
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(webots_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('odometry/filtered', '/odom')],
        condition=IfCondition(LaunchConfiguration('ekf')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(webots_share, 'launch', 'navigation.launch.py')),
        condition=IfCondition(LaunchConfiguration('nav')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(webots_share, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    shutdown_on_webots_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='false'),
        webots,
        agrobot_driver,
        robot_state_publisher,
        ekf_node,
        navigation,
        rviz,
        shutdown_on_webots_exit,
    ])
