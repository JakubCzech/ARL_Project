from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_aruco",
                executable="aruco_node",
                name="aruco_node",
            ),
            Node(
                package="tello_arl",
                executable="controller",
                name="controller",
            ),
        ]
    )
