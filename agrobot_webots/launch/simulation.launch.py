import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
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

    agrobot_driver = WebotsController(
        robot_name='agrobot',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen',
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
        webots,
        agrobot_driver,
        robot_state_publisher,
        navigation,
        rviz,
        shutdown_on_webots_exit,
    ])
