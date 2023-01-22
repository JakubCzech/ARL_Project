"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

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
    ns2 = "drone2"
    world_path = os.path.join(get_package_share_directory("tello_gazebo"), "worlds", "empty.world")
    urdf_path = os.path.join(get_package_share_directory("tello_description"), "urdf", "tello_1.urdf")
    urdf_path2 = os.path.join(get_package_share_directory("tello_description_arl"), "urdf", "tello_2.urdf")
    image_topic = "/drone1/image_raw"
    camera_info_topic = "/drone1/camera_info"
    aruco_poses_topic = "/aruco_poses"
    aruco_markers_topic = "/aruco_markers"
    cmd_vel_topic = "/drone1/cmd_vel"
    flight_data_topic = "/drone1/flight_data"
    offset_x = 0.025
    offset_y = 0.025
    offset_z = 0.025
    offset_R = 0.15

    drone_state_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_path],
        namespace=ns2,
    )
    drone_state_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_path2],
        namespace=ns2,
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
    drone_1_take_off = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/drone1/tello_action ",
                "tello_msgs/TelloAction ",
                '"{cmd: takeoff}"',
            ]
        ],
        shell=True,
    )
    drone_2_take_off = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/drone2/tello_action ",
                "tello_msgs/TelloAction ",
                '"{cmd: takeoff}"',
            ]
        ],
        shell=True,
    )
    inject_entity_1 = Node(
        package="tello_gazebo",
        executable="inject_entity.py",
        output="screen",
        arguments=[urdf_path, "0", "0", "1", "0"],
    )
    inject_entity_2 = Node(
        package="tello_gazebo",
        executable="inject_entity.py",
        output="screen",
        arguments=[urdf_path2, "1", "0", "1", "0"],
    )
    tello_driver_1 = Node(
        package="tello_driver",
        executable="tello_joy_main",
        output="screen",
        namespace=ns,
    )
    tello_driver_2 = Node(
        package="tello_driver",
        executable="tello_joy_main",
        output="screen",
        namespace=ns2,
    )
    aruco_ = Node(
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
    )
    controller_ = Node(
        package="tello_arl",
        executable="controller",
        name="controller",
        parameters=[
            {"flight_data_topic": flight_data_topic},
            {"aruco_topic": aruco_poses_topic},
            {"cmd_vel_topic": cmd_vel_topic},
            {"log_level": 50},
            {"speed_horizontal": 0.2},
            {"speed_vertical": 0.2},
            {"speed_front": 0.25},
            {"speed_back": 0.15},
            {"speed_angular": 0.15},
            {"distance": 0.75},
            {"offset_x": offset_x},
            {"offset_y": offset_y},
            {"offset_z": offset_z},
            {"offset_rotation": offset_R},
            {"frequency": 10.0},
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
            inject_entity_1,
            drone_state_1,
            tello_driver_1,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=tello_driver_1,
                    on_start=[aruco_],
                )
            ),
            inject_entity_2,
            drone_state_2,
            # Joystick driver, generates /namespace/joy messages
            Node(package="joy", executable="joy_node", output="screen", namespace=ns2),
            # Joystick controller, generates /namespace/cmd_vel messages
            tello_driver_2,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=tello_driver_2,
                    on_start=[drone_2_take_off],
                )
            ),
            # RegisterEventHandler(
            #     OnProcessStart(
            #         target_action=tello_driver_1,
            #         on_start=[drone_1_take_off],
            #     )
            # ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=drone_state_1,
                    on_start=[controller_, image, visualization_],
                )
            ),
        ]
    )
