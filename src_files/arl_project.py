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
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown,
)


def generate_launch_description():
    ns = "drone1"
    ns2 = "drone2"
    world_path = os.path.join(
        get_package_share_directory("tello_gazebo"), "worlds", "f2.world"
    )
    urdf_path = os.path.join(
        get_package_share_directory("tello_description"), "urdf", "tello_1.urdf"
    )
    urdf_path2 = os.path.join(
        get_package_share_directory("tello_description_arl"), "urdf", "tello_2.urdf"
    )

    drone_state_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_path],
    )
    drone_state_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf_path2],
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
        arguments=[urdf_path2, "1", "1", "1", "0"],
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

    return LaunchDescription(
        [
            start_gazebo,
            inject_entity_1,
            inject_entity_2,
            # Publish static transforms
            drone_state_1,
            drone_state_2,
            # Joystick driver, generates /namespace/joy messages
            Node(package="joy", executable="joy_node", output="screen", namespace=ns2),
            # Joystick controller, generates /namespace/cmd_vel messages
            tello_driver_1,
            tello_driver_2,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=tello_driver_1, on_start=[drone_1_take_off]
                )
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=tello_driver_2, on_start=[drone_2_take_off]
                )
            ),
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
