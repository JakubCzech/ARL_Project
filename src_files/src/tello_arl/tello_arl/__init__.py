#!/usr/bin/env python3

from rclpy.node import Node
from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import Image
from scipy.spatial import transform

# from simple_pid import PID ### TODO: implement PID controller


class TelloARL(Node):
    def __init__(self):
        super().__init__("tello_arl")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_topic", "/cmd_vel"),
                ("aruco_topic", "/aruco_poses"),
                ("flight_data_topic", "/flight_data"),
                ("log_level", 10),
                ("speed_horizontal", 15.0),
                ("speed_vertical", 10.0),
                ("speed_frontal", 15.0),
                ("speed_angular", 30.0),
                ("distance", 10.0),
                ("offset_x", 0.2),
                ("offset_y", 0.2),
                ("offset_z", 0.2),
                ("offset_rotation", 0.2),
                ("frequency", 1.0),
                ("velocity_send_method", "ros_service"),
                ("service_name", "/tello_action"),
                ("limit_linear", 25.0),
                ("limit_angular", 50.0),
                ("simulation", False),
            ],
        )
        self.__init_variables()
        self.sim = self.get_parameter("simulation").get_parameter_value().bool_value

        self.speed_limit_linear = (
            self.get_parameter("limit_linear").get_parameter_value().double_value
        )
        self.speed_limit_angular = (
            self.get_parameter("limit_linear").get_parameter_value().double_value
        )

        self.distance = (
            self.get_parameter("distance").get_parameter_value().double_value
        )

        self.speed_horizontal = (
            self.get_parameter("speed_horizontal").get_parameter_value().double_value
        )
        self.speed_vertical = (
            self.get_parameter("speed_vertical").get_parameter_value().double_value
        )
        self.speed_frontal = (
            self.get_parameter("speed_frontal").get_parameter_value().double_value
        )

        self.angular_speed = (
            self.get_parameter("speed_angular").get_parameter_value().double_value
        )

        self.offset_x = (
            self.get_parameter("offset_x").get_parameter_value().double_value
        )
        self.offset_y = (
            self.get_parameter("offset_y").get_parameter_value().double_value
        )
        self.offset_z = (
            self.get_parameter("offset_z").get_parameter_value().double_value
        )
        self.offset_rotation = (
            self.get_parameter("offset_rotation").get_parameter_value().double_value
        )

        self._logger.set_level(
            self.get_parameter("log_level").get_parameter_value().integer_value
        )

        self.sending_method = (
            self.get_parameter("velocity_send_method")
            .get_parameter_value()
            .string_value
        )

        self._sub_flight_data = self.create_subscription(
            FlightData,
            self.get_parameter("flight_data_topic").get_parameter_value().string_value,
            self.flight_data_callback,
            10,
        )
        self._sub_aruco_pose = self.create_subscription(
            PoseArray,
            self.get_parameter("aruco_topic").get_parameter_value().string_value,
            self.aruco_pose_callback,
            10,
        )

        self._pub_cmd_vel = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10,
        )

        self._call_service = self.create_client(
            TelloAction,
            self.get_parameter("service_name").get_parameter_value().string_value,
        )

        while not self._call_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("service not available, waiting again...")
        self._response = self.create_subscription(
            TelloResponse, "tello_response", self.response_callback, 10
        )

        self.waiting_response = False

        self.create_timer(
            1.0 / self.get_parameter("frequency").get_parameter_value().double_value,
            self.timer_callback,
        )
        self._call_service.call_async(TelloAction.Request(cmd="takeoff"))

        self.get_logger().info("TelloARL node has been started")

    def __init_variables(self):
        self._flight_data = FlightData()
        self._twist = Twist()
        self._image = Image()
        self._aruco_pose = Pose()
        self.__new_cmd = None
        self.last_direction = None
        self.pitch = None

    def __del__(self):
        self.get_logger().info(
            f"Battery: {self._flight_data.bat}, Height: {self._flight_data.h}"
        )
        self._pub_cmd_vel.publish(Twist())
        req = TelloAction.Request()
        req.cmd = f"rc 0 0 0 0"
        self._call_service.call_async(req)
        self._call_service.call_async(TelloAction.Request(cmd="land"))
        self.get_logger().info("TelloARL node has been stopped")

    ########################### Callbacks #######################################

    def response_callback(self, msg: TelloResponse):
        self.get_logger().info(f"Response: {msg.rc}")

    def flight_data_callback(self, msg: FlightData):
        self.get_logger().debug("Flight data callback")
        self._flight_data = msg

    def aruco_pose_callback(self, msg):
        self._aruco_pose = msg.poses[0]

        _, self.pitch, _ = transform.Rotation.from_quat(
            [
                self._aruco_pose.orientation.x,
                self._aruco_pose.orientation.y,
                self._aruco_pose.orientation.z,
                self._aruco_pose.orientation.w,
            ]
        ).as_euler("xyz")

        # Z velocity vertical
        self._twist.linear.z = self.__set_direction(
            self.offset_y, self.speed_horizontal, self._aruco_pose.position.y, 0.0
        )
        self._twist.linear.z *= -1
        # Y velocity horizontal
        self._twist.linear.y = self.__set_direction(
            self.offset_x, self.speed_vertical, self._aruco_pose.position.x, 0.0
        )
        # Angular velocity
        self._twist.angular.z = self.__set_direction(
            self.offset_rotation, self.angular_speed / 10, self.pitch, 0.0
        )
        self._twist.angular.z *= -1
        # X velocity forward/backward
        self._twist.linear.x = self.__set_direction(
            self.offset_z,
            self.speed_frontal,
            self._aruco_pose.position.z,
            self.distance,
        )
        self.__set_limit_speed()
        self.__new_cmd = True

    def timer_callback(self):
        self.get_logger().debug(
            f"Timer callback. Battery: {self._flight_data.bat}, Height: {self._flight_data.h}"
        )

        if self.__new_cmd:
            self.__send_cmd()
            self.__new_cmd = False
        else:
            self.get_logger().debug("No new command")
            self._twist = Twist()
            self.__send_cmd()

    ########################### Velocity #######################################

    def __set_direction(
        self, __offset: float, __speed: float, __dist: float, __distance: float = 0.0
    ):
        if -__offset + __distance < __dist < __offset + __distance:
            return 0.0
        if __dist >= __distance + __offset:
            return self.__set_velocity(__offset, __speed, __dist, __distance)
        if __dist <= __distance - __offset:
            return -self.__set_velocity(__offset, __speed, __dist, __distance)

        return 0.0

    def __set_velocity(
        self, __offset: float, __speed: float, __dist: float, __distance: float = 0.0
    ):
        if abs(__dist) > 3 * (__offset + __distance):
            return __speed * 3

        if abs(__dist) > 2 * __offset + __distance:
            return __speed * abs(__dist) / (__offset + __distance)
        else:
            return __speed * abs(__dist) / (__offset + __distance) / 2

    def __set_limit_speed(self):

        if abs(self._twist.linear.x) > self.speed_limit_linear:
            self._twist.linear.x = (
                self.speed_limit_linear
                * abs(self._twist.linear.x)
                / self._twist.linear.x
            )

        if abs(self._twist.linear.y) > self.speed_limit_linear:
            self._twist.linear.y = (
                self.speed_limit_linear
                * abs(self._twist.linear.y)
                / self._twist.linear.y
            )
        if abs(self._twist.linear.z) > self.speed_limit_linear:
            self._twist.linear.z = (
                self.speed_limit_linear
                * abs(self._twist.linear.z)
                / self._twist.linear.z
            )

        if abs(self._twist.angular.z) > self.speed_limit_angular:
            self._twist.angular.z = (
                self.speed_limit_angular
                * abs(self._twist.angular.z)
                / self._twist.angular.z
            )

    def __send_cmd(self):

        self.get_logger().debug(
            f"Aruco pose. Y_VEL:{self._aruco_pose.position.x}, Z_VEL:{self._aruco_pose.position.y}, Distance:{self._aruco_pose.position.z}, Rotatation{self.pitch}"
        )
        self.get_logger().debug(
            f"Drone speed. Y_VEL:{self._twist.linear.y} X_VEL:{self._twist.linear.x} Z_VEL:{self._twist.linear.z} ROT_VEL:{self._twist.angular.z}"
        )

        if self.sending_method == "ros_service":
            req = TelloAction.Request()
            req.cmd = f"rc {int(self._twist.linear.y)} {int(self._twist.linear.x)} {int(self._twist.linear.z)} {int(self._twist.angular.z)}"
            self._pub_cmd_vel.publish(self._twist)

            self._call_service.call_async(req)
            self.get_logger().debug(f"{req.cmd}")
        if self.sim:
            self._twist.linear.y *= -1

        self._pub_cmd_vel.publish(self._twist)
        self.get_logger().debug(f"{self._twist}")
