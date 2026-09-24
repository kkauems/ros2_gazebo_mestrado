import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_share = get_package_share_directory("agrobot_gazebo")
    description_share = get_package_share_directory("agrobot_description")
    world_path = os.path.join(gazebo_share, "obstacle_arena_world.sdf")
    urdf_path = os.path.join(description_share, "urdf", "agrobot.urdf")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        output="screen"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "obstacle_arena_world",
                    "-file",
                    urdf_path,
                    "-name", "agrobot",
                    "-x", "0",
                    "-y", "0",
                    "-z", "0.30",
                ],
                output="screen",
            )
        ],
    )

    bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                ],
                parameters=[{"lazy": False}],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])