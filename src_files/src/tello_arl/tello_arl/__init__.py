#!/usr/bin/env python3

from rclpy.node import Node
from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import Image
from scipy.spatial import transform
import os


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
                ("speed_linear", 0.3),
                ("speed_angular", 0.3),
                ("distance", 10.0),
                ("offset", 0.2),
                ("offset_rotation", 0.1),
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

        self.linear_speed = (
            self.get_parameter("speed_linear").get_parameter_value().double_value
        )

        self.angular_speed = (
            self.get_parameter("speed_angular").get_parameter_value().double_value
        )

        self.offset = self.get_parameter("offset").get_parameter_value().double_value
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

        self.__set_x_velocity()
        self.__set_y_velocity()
        self.__set_z_velocity()
        self.__set_angular_velocity()
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

    ########################### SPEED SETTING #######################################
    def __set_limit_speed(self):
        if self._twist.linear.x > self.speed_limit_linear:
            self._twist.linear.x = self.speed_limit_linear
        if self._twist.linear.y > self.speed_limit_linear:
            self._twist.linear.y = self.speed_limit_linear
        if self._twist.linear.z > self.speed_limit_linear:
            self._twist.linear.z = self.speed_limit_linear
        if self._twist.angular.z > self.speed_limit_angular:
            self._twist.angular.z = self.speed_limit_angular

        if self._twist.linear.x < -self.speed_limit_linear:
            self._twist.linear.x = -self.speed_limit_linear
        if self._twist.linear.y < -self.speed_limit_linear:
            self._twist.linear.y = -self.speed_limit_linear
        if self._twist.linear.z < -self.speed_limit_linear:
            self._twist.linear.z = -self.speed_limit_linear
        if self._twist.angular.z < -self.speed_limit_angular:
            self._twist.angular.z = -self.speed_limit_angular

    def __set_angular_velocity(self):
        if -self.offset_rotation < self.pitch < self.offset_rotation:
            self._twist.angular.z = 0.0
            self.get_logger().debug(f"Pitch: {self.pitch}")
        elif self.pitch > self.offset_rotation:
            self.get_logger().debug(f"Rotate right {self.pitch}")
            self._twist.angular.z = self.angular_speed * self.pitch**2
        elif self.pitch < -self.offset_rotation:
            self.get_logger().debug(f"Rotate left {self.pitch}")
            self._twist.angular.z = -self.angular_speed * self.pitch**2

    def __set_x_velocity(self):
        __dist = self._aruco_pose.position.z

        if (self.distance - self.offset) < __dist < (self.distance + self.offset):
            self._twist.linear.x = 0.0
            self.get_logger().debug(f"X distance: {__dist}")
        elif __dist > (self.distance + self.offset):
            self.get_logger().debug(f"X distance: {__dist}, Move forward")
            __dist -= self.distance
            self._twist.linear.x = self.linear_speed
        elif __dist < (self.distance - self.offset):
            self.get_logger().debug(f"X distance: {__dist}, Move backward")
            __dist += self.distance
            self._twist.linear.x = -self.linear_speed

    def __set_y_velocity(self):
        __dist = self._aruco_pose.position.x

        if -self.offset < __dist < self.offset:
            self._twist.linear.y = 0.0
            self.get_logger().debug(f"Y distance: {__dist}")
        elif __dist < self.offset:
            self._twist.linear.y = -self.linear_speed
            self.get_logger().debug(f"Y distance: {__dist}, Move right")
        elif __dist > -self.offset:
            self._twist.linear.y = self.linear_speed
            self.get_logger().debug(f"Y distance: {__dist}, Move left")

    def __set_z_velocity(self):
        __dist = self._aruco_pose.position.y

        if -self.offset / 2 < __dist < self.offset / 2:
            self._twist.linear.z = 0.0
            self.get_logger().debug(f"Z distance: {__dist}")
        elif __dist > self.offset / 2:
            self.get_logger().debug(f"Z distance: {__dist}, Move down")
            self._twist.linear.z = -self.linear_speed
        elif __dist < -self.offset / 2:
            self.get_logger().debug(f"Z distance: {__dist}, Move up")
            self._twist.linear.z = self.linear_speed

    def __send_cmd(self):

        self.get_logger().debug(
            f"Aruco pose. Y_VEL:{self._aruco_pose.position.x}, Z_VEL:{self._aruco_pose.position.y}, Distance:{self._aruco_pose.position.z}, Rotatation{self.pitch}"
        )

        self.get_logger().debug(
            f"Drone speed. Y_VEL:{self._twist.linear.y*10 } X_VEL:{self._twist.linear.x*15 } Z_VEL:{self._twist.linear.z*15 } ROT_VEL:{self._twist.angular.z }"
        )

        if self.sending_method == "ros_service":
            req = TelloAction.Request()
            req.cmd = f"rc {int(self._twist.linear.y*10 )} {int(self._twist.linear.x*15 )} {int(self._twist.linear.z*15 )} {int(-1*self._twist.angular.z*30 )}"
            self._call_service.call_async(req)
            self.get_logger().debug(f"{req.cmd}")
        elif self.sending_method == "ros_topic":
            if self.sim:
                self._twist.linear.y *= -1
                self._twist.angular.z *= -1

            self._pub_cmd_vel.publish(self._twist)
            self.get_logger().debug(f"{self._twist}")

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
