from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_aruco",
                executable="aruco_node",
                name="aruco_node",
                parameters=[
                    {"marker_size": 0.0625},
                    {"aruco_dictionary_id": "DICT_6X6_100"},
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
                    {"speed": 5.0},  # 100.0 / 5.0
                    {"distance": 0.5},
                    {"offset": 0.1},
                    {"offset_rotation": 0.1},
                    {"frequency": 10.0},
                ],
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                output="screen",
                arguments=["/image_raw"],
            ),
        ]
    )
