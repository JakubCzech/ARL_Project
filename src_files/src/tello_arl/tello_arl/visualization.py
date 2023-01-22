#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tello_msgs.msg import FlightData
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import Image
from scipy.spatial import transform
from cv_bridge import CvBridge, CvBridgeError
import cv2

BAD_COLOR = (35, 22, 148)
GOOD_COLOR = (148, 26, 22)


class Visualization(Node):
    def __init__(self):
        super().__init__("visualization")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_topic", "/cmd_vel"),
                ("aruco_topic", "/aruco_poses"),
                ("flight_data_topic", "/flight_data"),
                ("offset_x", 0.2),
                ("offset_y", 0.2),
                ("offset_z", 0.2),
                ("offset_rotation", 0.1),
                ("camera_topic", "drone1/image_raw"),
                ("distance", 1.0),
            ],
        )
        self.__init_variables()

        self.distance = (
            self.get_parameter("distance").get_parameter_value().double_value
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
        self._sub_aruco_pose = self.create_subscription(
            PoseArray,
            self.get_parameter("aruco_topic").get_parameter_value().string_value,
            self.aruco_pose_callback,
            10,
        )
        self._sub_cmd_vel = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            self.cmd_vel_callback,
            10,
        )
        self._sub_flight_data = self.create_subscription(
            FlightData,
            self.get_parameter("flight_data_topic").get_parameter_value().string_value,
            self.flight_data_callback,
            10,
        )
        self._sub_cam = self.create_subscription(
            Image,
            self.get_parameter("camera_topic").get_parameter_value().string_value,
            self.camera_callback,
            10,
        )

        self._pub_image = self.create_publisher(
            Image,
            "visualization",
            10,
        )

        self.get_logger().info("TelloARL node has been started")

    def __init_variables(self):
        self._flight_data = FlightData()
        self._twist = Twist()
        self._image = Image()
        self._aruco_pose = Pose()
        self.pitch = 0.0

    ########################### Callbacks #######################################

    def camera_callback(self, msg: Image):
        self.get_logger().debug("Camera callback")

        ##### Convert #####
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            cv_image = bridge.imgmsg_to_cv2(msg, "mono8")

        ##### Default text #####
        font = (cv2.FONT_HERSHEY_SIMPLEX, 0.8, GOOD_COLOR, 1, cv2.LINE_AA)
        cv_image = cv2.putText(
            cv_image,
            f"VEL:",
            (10, 30),
            *font,
        )
        cv_image = cv2.putText(
            cv_image,
            f"AR:",
            (10, 80),
            *font,
        )

        ##### Speed Params #####
        if self._twist is not None:
            self.draw_twist_params(cv_image, font)

        __dist = 300

        ##### Aruco Params #####
        if self._aruco_pose is not None:
            if self._aruco_pose.position.z > 0:
                __dist /= self._aruco_pose.position.z
            self.draw_aruco_params(cv_image)

        ##### Flight Params #####
        if self._flight_data is not None:
            cv_image = cv2.putText(
                cv_image,
                f"BAT: {self._flight_data.bat:.2f}",
                (cv_image.shape[1] - 200, 30),
                *font,
            )

        ##### Aruco Position #####
        if self._aruco_pose is not None:
            cv_image = cv2.circle(
                cv_image,
                (
                    int(cv_image.shape[1] / 2 + self._aruco_pose.position.x * __dist),
                    int(cv_image.shape[0] / 2 + self._aruco_pose.position.y * __dist),
                ),
                5,
                GOOD_COLOR,
                -1,
            )

        ##### Border #####
        self.draw_border(cv_image, __dist)

        ##### Convert to ros #####
        _image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self._last_pose = self._aruco_pose
        self._pub_image.publish(_image)

    def flight_data_callback(self, msg: FlightData):
        self.get_logger().debug("Flight data callback")
        self._flight_data = msg

    def aruco_pose_callback(self, msg: PoseArray):
        self._aruco_pose = msg.poses[0]

        _, self.pitch, _ = transform.Rotation.from_quat(
            [
                self._aruco_pose.orientation.x,
                self._aruco_pose.orientation.y,
                self._aruco_pose.orientation.z,
                self._aruco_pose.orientation.w,
            ]
        ).as_euler("xyz")

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().debug("Cmd vel callback")
        self._twist = msg

    ########################### Draw functions #######################################

    def draw_aruco_params(self, cv_image):
        for ar_pos, position, offset in zip(
            [
                self._aruco_pose.position.z,
                self._aruco_pose.position.x,
                self._aruco_pose.position.y,
                self.pitch,
            ],
            [80, 170, 260, 350],
            [self.offset_x, self.offset_y, self.offset_z, self.offset_rotation],
        ):

            if abs(ar_pos) > offset and position != 80:
                font = (cv2.FONT_HERSHEY_SIMPLEX, 0.8, BAD_COLOR, 1, cv2.LINE_AA)
            elif (
                abs(ar_pos) > offset + self.distance
                or abs(ar_pos) < -offset + self.distance
            ) and position == 80:
                font = (cv2.FONT_HERSHEY_SIMPLEX, 0.8, BAD_COLOR, 1, cv2.LINE_AA)
            else:
                font = (cv2.FONT_HERSHEY_SIMPLEX, 0.8, GOOD_COLOR, 1, cv2.LINE_AA)

            __text = f"{ar_pos:.2f}"
            if ar_pos > 0:
                __text = f"+{ar_pos:.2f}"

            cv_image = cv2.putText(
                cv_image,
                __text,
                (position, 80),
                *font,
            )

    def draw_twist_params(self, cv_image, font):
        for speed, position in zip(
            [
                self._twist.linear.x,
                self._twist.linear.y,
                self._twist.linear.z,
                self._twist.angular.z,
            ],
            [80, 170, 260, 350],
        ):

            __text = f"{speed:.2f}"
            if speed > 0:
                __text = f"+{speed:.2f}"

            cv_image = cv2.putText(
                cv_image,
                __text,
                (position, 30),
                *font,
            )

    def draw_border(self, cv_image, __dist):
        border = (BAD_COLOR, 1)
        cv_image = cv2.line(
            cv_image,
            (0, int(cv_image.shape[0] / 2 + self.offset_x * __dist)),
            (
                cv_image.shape[1],
                int(cv_image.shape[0] / 2 + self.offset_x * __dist),
            ),
            *border,
        )
        cv_image = cv2.line(
            cv_image,
            (0, int(cv_image.shape[0] / 2 - self.offset_x * __dist)),
            (
                cv_image.shape[1],
                int(cv_image.shape[0] / 2 - self.offset_x * __dist),
            ),
            *border,
        )
        cv_image = cv2.line(
            cv_image,
            (int(cv_image.shape[1] / 2 + self.offset_y * __dist), 0),
            (
                int(cv_image.shape[1] / 2 + self.offset_y * __dist),
                cv_image.shape[0],
            ),
            *border,
        )
        cv_image = cv2.line(
            cv_image,
            (int(cv_image.shape[1] / 2 - self.offset_y * __dist), 0),
            (
                int(cv_image.shape[1] / 2 - self.offset_y * __dist),
                cv_image.shape[0],
            ),
            *border,
        )


def main(args=None):
    rclpy.init(args=args)
    visualization = Visualization()

    try:
        rclpy.spin(visualization)
    except KeyboardInterrupt:
        pass
    finally:
        visualization.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(args=None)
