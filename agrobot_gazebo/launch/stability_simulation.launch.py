from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    world_path = (
        "agrobot_gazebo/stability_test_world.sdf"
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        output="screen"
    )

    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "stability_test_world",
                    "-file",
                    "agrobot_description/urdf/agrobot.urdf",
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
                    "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
                ],
                parameters=[{"lazy": False}],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge,
    ])