#!/usr/bin/env python3

from rclpy.node import Node
from tello_msgs.msg import FlightData
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import Image
from scipy.spatial import transform


class TelloARL(Node):
    def __init__(self):
        super().__init__("tello_arl")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_topic", None),
                ("aruco_topic", None),
                ("flight_data_topic", None),
                ("log_level", None),
                ("speed", None),
                ("distance", None),
                ("offset_rotation", None),
                ("frequency", None),
                ("offset", None),
            ],
        )
        self.__init_variables()

        self.offset = self.get_parameter("offset").get_parameter_value().double_value
        self.distance = (
            self.get_parameter("distance").get_parameter_value().double_value
        )
        self.linear_speed = (
            100.0 * self.get_parameter("speed").get_parameter_value().double_value
        )

        self.angular_speed = (
            100.0 * self.get_parameter("speed").get_parameter_value().double_value
        )

        self.offset_rotation = (
            self.get_parameter("offset_rotation").get_parameter_value().double_value
        )
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )

        self._logger.set_level(
            self.get_parameter("log_level").get_parameter_value().integer_value
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
        self.create_timer(
            1.0 / self.get_parameter("frequency").get_parameter_value().double_value,
            self.timer_callback,
        )
        self.get_logger().info("TelloARL node has been started")

    def __init_variables(self):
        self._flight_data = FlightData()
        self._twist = Twist()
        self._image = Image()
        self._aruco_pose = Pose()
        self.__new_cmd = None
        self.last_direction = None

    def camera_callback(self, msg):
        self.get_logger().debug("Camera callback")
        self._image = msg

    def flight_data_callback(self, msg):
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
        self.get_logger().info(
            f"New aruco pose: {self._aruco_pose.position.x}, {self._aruco_pose.position.y}, {self._aruco_pose.position.z}, {self.pitch}"
        )

        self.__set_linear_velocity()
        self.__set_angular_velocity()
        self.__new_cmd = True

    def timer_callback(self):
        self.get_logger().debug("Timer callback")
        self.get_logger().debug(
            f"Battery: {self._flight_data.bat}, Height: {self._flight_data.h}"
        )
        if self.__new_cmd:
            self.__send_cmd()
            self.__new_cmd = False
        else:
            self._twist = Twist()
            if self.last_direction:
                self._twist.angular.z = self.last_direction
            else:
                self._twist.angular.z = -0.005
            self.__send_cmd()

    def __set_angular_velocity(self):
        if self.pitch != 0.0:
            if self.pitch > self.offset_rotation:
                self.get_logger().debug("Turn left")
                self._twist.angular.z = -self.angular_speed / 2
                self.last_direction = -self.angular_speed
            elif self.pitch < -self.offset_rotation:
                self.get_logger().debug("Turn right")
                self._twist.angular.z = +self.angular_speed / 2
                self.last_direction = +self.angular_speed
            else:
                self._twist.angular.z = 0.0
                self.get_logger().debug("Stop")

    def __set_linear_velocity(self):
        if self._aruco_pose.position.z != 0.0:
            if self._aruco_pose.position.z > self.distance + self.offset:
                self.get_logger().debug("Move front")
                if self._aruco_pose.position.z > 2.5:
                    self._twist.linear.x = self.linear_speed * (
                        self._aruco_pose.position.z / 2
                    )
                else:
                    self._twist.linear.x = self.linear_speed
            elif self._aruco_pose.position.z < self.distance - self.offset:
                self.get_logger().debug("Move back")
                if self._aruco_pose.position.z < 2.5:
                    self._twist.linear.x = self.linear_speed * (
                        self._aruco_pose.position.z / 2
                    )
                else:
                    self._twist.linear.x = -self.linear_speed
            else:
                self._twist.linear.x = 0.0
                self.get_logger().debug("Stop")
        else:
            self._twist.linear.x = 0.0
            self.get_logger().debug("Stop")

        if self._aruco_pose.position.x != 0.0:
            if self._aruco_pose.position.x < -self.offset:
                self.get_logger().debug("Move left")
                self._twist.linear.y = self.linear_speed
            elif self._aruco_pose.position.x > self.offset:
                self.get_logger().debug("Move right")
                self._twist.linear.y = -self.linear_speed
            else:
                self._twist.linear.y = 0.0
                self.get_logger().debug("Stop")
        else:
            self._twist.linear.y = 0.0
            self.get_logger().debug("Stop")

        if self._aruco_pose.position.y != 0.0:
            if self._aruco_pose.position.y < -self.offset:
                self.get_logger().debug("Move up")
                self._twist.linear.z = self.linear_speed
            elif self._aruco_pose.position.y > self.offset:
                self.get_logger().debug("Move down")
                self._twist.linear.z = -self.linear_speed
            else:
                self._twist.linear.z = 0.0
                self.get_logger().debug("Stop")

        else:
            self._twist.linear.z = 0.0
            self.get_logger().debug("Stop")

    def __send_cmd(self):
        self._pub_cmd_vel.publish(self._twist)
        self.get_logger().debug(f"Distance to aruco: {self._aruco_pose.position.z}")

    def __del__(self):
        self.get_logger().debug("TelloARL node has been stopped")
        self._pub_cmd_vel.publish(Twist())
