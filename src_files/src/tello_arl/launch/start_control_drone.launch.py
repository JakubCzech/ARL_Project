from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_ = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        parameters=[
            {"marker_size": 0.0625},
            {"aruco_dictionary_id": "DICT_6X6_100"},
            {"image_topic": "/drone1/image_raw"},
            {"camera_info_topic": "/drone1/camera_info"},
        ],
    )
    controller_ = Node(
        package="tello_arl",
        executable="controller",
        name="controller",
        parameters=[
            {"aruco_topic": "/drone1/aruco_poses"},
            {"cmd_vel_topic": "/drone1/cmd_vel"},
            {"flight_data_topic": "/drone1/flight_data"},
            {"log_level": 40},
            {"speed": 000.1},
            {"distance": 1.0},
            {"offset": 0.05},
            {"offset_rotation": 0.05},
            {"frequency": 10.0},
        ],
    )

    return LaunchDescription(
        [
            aruco_,
            controller_,
        ]
    )
