"""Webots + RViz + goal_navigator: objetivo pelo "2D Goal Pose" do RViz.

Sem Nav2 e sem mapa. Reaproveita simulation.launch.py (Webots, driver do
Agrobot e robot_state_publisher) com nav:=false, e acrescenta:
- o RViz com rviz/goal.rviz (Fixed Frame odom, ferramenta 2D Goal Pose);
- o nó goal_navigator, que leva o robô até o objetivo usando só a
  odometria e desvia de obstáculos diretamente à frente.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    webots_share = get_package_share_directory('agrobot_webots')

    # O GroupAction isola os argumentos passados ao include. Sem ele, o
    # rviz:=false do simulation.launch.py vaza para este arquivo e desliga
    # também o RViz daqui.
    simulation = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(webots_share, 'launch', 'simulation.launch.py')),
            launch_arguments={'nav': 'false', 'rviz': 'false'}.items(),
        ),
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(webots_share, 'rviz', 'goal.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    goal_navigator = Node(
        package='agrobot_webots',
        executable='goal_navigator',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        simulation,
        rviz,
        goal_navigator,
    ])
