import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
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

    agrobot_driver = WebotsController(
        robot_name='agrobot',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        webots,
        agrobot_driver,
    ])