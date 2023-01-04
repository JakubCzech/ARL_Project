import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         [
            #             PathJoinSubstitution(
            #                 [
            #                     FindPackageShare("tello_driver"),
            #                     "launch",
            #                     "teleop_launch.py",
            #                 ]
            #             )
            #         ]
            #     ),
            # ),
            Node(
                package="ros2_aruco",
                executable="aruco_node",
                name="aruco_node",
                parameters=[
                    {"marker_size": 0.07},
                    {"aruco_dictionary_id": "DICT_ARUCO_ORIGINAL"},
                    {"image_topic": "/image_raw"},
                    {"camera_info_topic": "/camera_info"},
                    {"aruco_poses_topic": "/aruco_poses"},
                    {"aruco_markers_topic": "/aruco_markers"},
                ],
            ),
            Node(
                package="tello_arl",
                executable="controller",
                name="controller",
                parameters=[
                    {"aruco_topic": "/aruco_poses"},
                    {"cmd_vel_topic": "/cmd_vel"},
                    {"flight_data_topic": "/flight_data"},
                    {"log_level": 20},
                    {"speed_linear": 1.0},  # 100.0 / 5.0
                    {"speed_angular": 10.0},  # 100.0 / 5.0
                    {"distance": 1.0},
                    {"offset": 0.1},
                    {"offset_rotation": 0.15},
                    {"limit_linear": 1.0},
                    {"limit_angular": 30.0},
                    {"frequency": 10.0},
                    {"velocity_send_method": "ros_service"},  # ros_service or ros_topic
                    {"service_name": "/tello_action"},
                    {"twist_real": -1},
                ],
            ),
            # Node(
            #     package="rqt_image_view",
            #     executable="rqt_image_view",
            #     output="screen",
            #     arguments=["/image_raw"],
            # ),
        ]
    )
