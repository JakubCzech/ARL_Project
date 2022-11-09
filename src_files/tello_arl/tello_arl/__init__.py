#!/usr/bin/env python3

from rclpy.node import Node
from tello_msgs.msg import FlightData
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import Image
import rclpy

OFFSET = 0.05
DISTANCE = 1.0
SPEED = 0.05


class TelloARL(Node):
    def __init__(self):
        super().__init__("tello_arl")
        self.__init_variables()
        self.get_logger().debug("TelloARL node has been started")
        # DEBUG, INFO, WARN, ERROR, FATAL debug for development, info for production
        self._logger.set_level(rclpy.logging.LoggingSeverity.INFO)
        self._sub_cam = self.create_subscription(
            Image,
            "/drone1/image_raw",
            self.camera_callback,
            10,
        )
        self._sub_flight_data = self.create_subscription(
            FlightData,
            "/drone1/flight_data",
            self.flight_data_callback,
            10,
        )
        self._sub_aruco_pose = self.create_subscription(
            PoseArray,
            "/drone1/aruco_poses",
            self.aruco_pose_callback,
            10,
        )
        self._pub_cmd_vel = self.create_publisher(Twist, "/drone1/cmd_vel", 10)
        self.create_timer(0.1, self.timer_callback)

    def __init_variables(self):
        self._flight_data = FlightData()
        self._twist = Twist()
        self._image = Image()
        self._aruco_pose = Pose()

    def camera_callback(self, msg):
        self.get_logger().debug("Camera callback")
        self._image = msg

    def flight_data_callback(self, msg):
        self.get_logger().debug("Flight data callback")
        self._flight_data = msg

    def aruco_pose_callback(self, msg):
        self.get_logger().debug("Aruco pose callback")
        self._aruco_pose = msg.poses[0]

    def timer_callback(self):
        self.get_logger().debug("Timer callback")
        self.get_logger().debug(
            f"Battery: {self._flight_data.bat}, Height: {self._flight_data.h}"
        )
        self.get_logger().debug(f"Aruco pose: {self._aruco_pose}")
        if self._aruco_pose.position.z != 0.0:
            if self._aruco_pose.position.z > DISTANCE + OFFSET:
                self.get_logger().info("Move front")
                self._twist.linear.x = SPEED
            elif self._aruco_pose.position.z < DISTANCE - OFFSET:
                self.get_logger().info("Move back")
                self._twist.linear.x = -SPEED
            else:
                self._twist.linear.x = 0.0
                self.get_logger().info("Stop")
        else:
            self._twist.linear.x = 0.0
            self.get_logger().info("Stop")

        if self._aruco_pose.position.x != 0.0:
            if self._aruco_pose.position.x < -OFFSET:
                self.get_logger().info("Move left")
                self._twist.linear.y = SPEED
            elif self._aruco_pose.position.x > OFFSET:
                self.get_logger().info("Move right")
                self._twist.linear.y = -SPEED
            else:
                self._twist.linear.y = 0.0
                self.get_logger().info("Stop")
        else:
            self._twist.linear.y = 0.0
            self.get_logger().info("Stop")

        if self._aruco_pose.position.y != 0.0:
            if self._aruco_pose.position.y < -OFFSET:
                self.get_logger().info("Move up")
                self._twist.linear.z = SPEED
            elif self._aruco_pose.position.y > OFFSET:
                self.get_logger().info("Move down")
                self._twist.linear.z = -SPEED
            else:
                self._twist.linear.z = 0.0
                self.get_logger().info("Stop")

        else:
            self._twist.linear.z = 0.0
            self.get_logger().info("Stop")

        self._pub_cmd_vel.publish(self._twist)
        self.get_logger().info(f"Distance to aruco: {self._aruco_pose.position.z}")

    def __del__(self):
        self.get_logger().debug("TelloARL node has been stopped")
        self._pub_cmd_vel.publish(Twist())
