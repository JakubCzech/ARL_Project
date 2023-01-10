from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = "/image_raw"
    camera_info_topic = "/camera_info"
    aruco_poses_topic = "/aruco_poses"
    aruco_markers_topic = "/aruco_markers"
    cmd_vel_topic = "/cmd_vel"
    flight_data_topic = "/flight_data"
    offset_x = 0.1
    offset_y = 0.1
    offset_z = 0.1
    offset_R = 0.15

    return LaunchDescription(
        [
            Node(
                package="ros2_aruco",
                executable="aruco_node",
                name="aruco_node",
                parameters=[
                    {"marker_size": 0.07},
                    {"aruco_dictionary_id": "DICT_ARUCO_ORIGINAL"},
                    {"image_topic": image_topic},
                    {"camera_info_topic": camera_info_topic},
                    {"aruco_poses_topic": aruco_poses_topic},
                    {"aruco_markers_topic": aruco_markers_topic},
                ],
            ),
            Node(
                package="tello_arl",
                executable="controller",
                name="controller",
                parameters=[
                    {"aruco_topic": aruco_poses_topic},
                    {"cmd_vel_topic": cmd_vel_topic},
                    {"flight_data_topic": flight_data_topic},
                    {"log_level": 20},
                    {"speed_horizontal": 15.0},
                    {"speed_vertical": 10.0},
                    {"speed_frontal": 15.0},
                    {"speed_angular": 30.0},
                    {"distance": 1.0},
                    {"offset_x": offset_x},
                    {"offset_y": offset_y},
                    {"offset_z": offset_z},
                    {"offset_rotation": offset_R},
                    {"limit_linear": 50.0},
                    {"limit_angular": 30.0},
                    {"frequency": 10.0},
                    {"velocity_send_method": "ros_service"},  # ros_service or ros_topic
                    {"service_name": "/tello_action"},
                ],
            ),
            Node(
                package="tello_arl",
                executable="visualization",
                name="visualization",
                parameters=[
                    {"aruco_topic": aruco_poses_topic},
                    {"flight_data_topic": flight_data_topic},
                    {"cmd_vel_topic": cmd_vel_topic},
                    {"camera_topic": image_topic},
                    {"offset_x": offset_x},
                    {"offset_y": offset_y},
                    {"offset_z": offset_z},
                    {"offset_rotation": offset_R},
                    {"distance": 1.0},
                ],
            ),
        ]
    )
