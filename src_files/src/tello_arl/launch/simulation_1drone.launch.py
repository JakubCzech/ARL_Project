import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import (
    OnProcessStart,
)


def generate_launch_description():
    ns = "drone1"
    world_path = os.path.join(
        get_package_share_directory("tello_gazebo"), "worlds", "f2.world"
    )
    urdf_path = os.path.join(
        get_package_share_directory("tello_description"), "urdf", "tello_1.urdf"
    )
    image_topic = "/drone1/image_raw"
    camera_info_topic = "/drone1/camera_info"
    aruco_poses_topic = "/aruco_poses"
    aruco_markers_topic = "/aruco_markers"
    cmd_vel_topic = "/drone1/cmd_vel"
    flight_data_topic = "/drone1/flight_data"
    offset_x = 0.1
    offset_y = 0.1
    offset_z = 0.1
    offset_R = 0.2

    drone_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_path],
    )
    image = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        output="screen",
        arguments=["visualization"],
    )
    start_gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",  # Publish /clock
            "-s",
            "libgazebo_ros_factory.so",  # Provide gazebo_ros::Node
            world_path,
        ],
        output="screen",
    )
    inject_entity = Node(
        package="tello_gazebo",
        executable="inject_entity.py",
        output="screen",
        arguments=[urdf_path, "0", "0", "1", "0"],
    )
    tello_driver = Node(
        package="tello_driver",
        executable="tello_joy_main",
        output="screen",
        namespace=ns,
    )
    aruco_ = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        parameters=[
            {"marker_size": 0.0625},
            {"aruco_dictionary_id": "DICT_6X6_100"},
            {"image_topic": image_topic},
            {"camera_info_topic": camera_info_topic},
            {"aruco_poses_topic": aruco_poses_topic},
            {"aruco_markers_topic": aruco_markers_topic},
        ],
    )
    controller_ = Node(
        package="tello_arl",
        executable="controller",
        name="controller",
        parameters=[
            {"aruco_topic": aruco_poses_topic},
            {"cmd_vel_topic": cmd_vel_topic},
            {"flight_data_topic": flight_data_topic},
            {"log_level": 20},
            {"speed_horizontal": 0.5},
            {"speed_vertical": 0.3},
            {"speed_frontal": 0.5},
            {"speed_angular": 1.0},
            {"distance": 0.75},
            {"offset_x": offset_x},
            {"offset_y": offset_y},
            {"offset_z": offset_z},
            {"offset_rotation": offset_R},
            {"frequency": 10.0},
            {"limit_linear": 1.5},
            {"limit_angular": 1.0},
            {"velocity_send_method": "ros_topic"},  # ros_service or ros_topic
            {"service_name": "/drone1/tello_action"},
            {"simulation": True},
        ],
    )
    visualization_ = Node(
        package="tello_arl",
        executable="visualization",
        name="visualization",
        parameters=[
            {"aruco_topic": aruco_poses_topic},
            {"flight_data_topic": flight_data_topic},
            {"cmd_vel_topic": cmd_vel_topic},
            {"offset_x": offset_x},
            {"offset_y": offset_y},
            {"offset_z": offset_z},
            {"offset_rotation": offset_R},
            {"distance": 0.75},
        ],
    )

    return LaunchDescription(
        [
            start_gazebo,
            inject_entity,
            drone_state,
            tello_driver,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=tello_driver,
                    on_start=[aruco_],
                )
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=aruco_,
                    on_start=[controller_, image, visualization_],
                )
            ),
        ]
    )
