import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory('agrobot_description'),
        'urdf',
        'agrobot.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'agrobot',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        )
    ])