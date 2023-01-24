#!/usr/bin/env python3

from rclpy.node import Node
from tello_msgs.msg import FlightData, TelloResponse
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import Image
from scipy.spatial import transform

from simple_pid import PID


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
                ("speed_horizontal", 40.0),
                ("speed_vertical", 30.0),
                ("speed_front", 50.0),
                ("speed_back", 20.0),
                ("speed_angular", 30.0),
                ("distance", 10.0),
                ("offset_x", 0.2),
                ("offset_y", 0.2),
                ("offset_z", 0.2),
                ("offset_rotation", 0.2),
                ("frequency", 1.0),
                ("velocity_send_method", "ros_service"),
                ("service_name", "/tello_action"),
                ("simulation", False),
            ],
        )
        self.__init_variables()
        self.sim = self.get_parameter("simulation").get_parameter_value().bool_value

        self.distance = self.get_parameter("distance").get_parameter_value().double_value

        self.speed_horizontal = self.get_parameter("speed_horizontal").get_parameter_value().double_value
        self.speed_vertical = self.get_parameter("speed_vertical").get_parameter_value().double_value
        self.speed_front = self.get_parameter("speed_front").get_parameter_value().double_value
        self.speed_back = self.get_parameter("speed_back").get_parameter_value().double_value

        self.angular_speed = self.get_parameter("speed_angular").get_parameter_value().double_value

        self.offset_x = self.get_parameter("offset_x").get_parameter_value().double_value
        self.offset_y = self.get_parameter("offset_y").get_parameter_value().double_value
        self.offset_z = self.get_parameter("offset_z").get_parameter_value().double_value
        self.offset_rotation = self.get_parameter("offset_rotation").get_parameter_value().double_value

        self._logger.set_level(self.get_parameter("log_level").get_parameter_value().integer_value)

        self.sending_method = self.get_parameter("velocity_send_method").get_parameter_value().string_value

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
        self._response = self.create_subscription(TelloResponse, "tello_response", self.response_callback, 10)

        self.waiting_response = False

        self.create_timer(
            1.0 / self.get_parameter("frequency").get_parameter_value().double_value,
            self.timer_callback,
        )
        self._call_service.call_async(TelloAction.Request(cmd="takeoff"))

        self.get_logger().info("TelloARL node has been started")

    def __init_variables(self):
        self._flight_data = FlightData()
        self.__new_cmd = None

    def __del__(self):
        self.get_logger().info(f"Battery: {self._flight_data.bat}, Height: {self._flight_data.h}")
        try:
            self._pub_cmd_vel.publish(Twist())
            req = TelloAction.Request()
            req.cmd = f"rc 0 0 0 0"
            self._call_service.call_async(req)
            self._call_service.call_async(TelloAction.Request(cmd="land"))
            self.get_logger().info("TelloARL node has been stopped")
        except Exception as e:
            pass

    ########################### Callbacks #######################################

    def response_callback(self, msg: TelloResponse):
        self.get_logger().info(f"Response: {msg.rc}")

    def flight_data_callback(self, msg: FlightData):
        self.get_logger().debug("Flight data callback")
        self._flight_data = msg

    def timer_callback(self):
        self.get_logger().debug(f"Timer callback. Battery: {self._flight_data.bat}, Height: {self._flight_data.h}")

        if self.__new_cmd:
            self.__send_cmd()
            self.__new_cmd = False
        else:
            self.get_logger().debug("No new command")
            self._twist = Twist()
            self.__send_cmd()

    def aruco_pose_callback(self, msg):
        aruco_pose = msg.poses[0]
        _, pitch, _ = transform.Rotation.from_quat(
            [
                aruco_pose.orientation.x,
                aruco_pose.orientation.y,
                aruco_pose.orientation.z,
                aruco_pose.orientation.w,
            ]
        ).as_euler("xyz")
        self._twist = Twist()

        # X velocity forward/backward
        if aruco_pose.position.z < self.distance - self.offset_z:
            self._twist.linear.x = -self.speed_front * (self.distance - aruco_pose.position.z)
            # self._twist.linear.x = -self.speed_frontal ## Oddalanie od znacznika ze stalą prędkością
        elif aruco_pose.position.z > self.distance + self.offset_z:
            self._twist.linear.x = self.__set_forward(self.speed_front, aruco_pose.position.z)

        # Y velocity horizontal
        if abs(aruco_pose.position.x) > self.offset_x:
            self._twist.linear.y = self.__set_velocity(self.offset_x, self.speed_vertical, aruco_pose.position.x)

        # Z velocity vertical
        if abs(aruco_pose.position.y) > self.offset_y:
            self._twist.linear.z = self.__set_velocity(self.offset_y, self.speed_horizontal, aruco_pose.position.y)

        # Angular velocity
        if abs(pitch) > self.offset_rotation:
            # self._twist.angular.z = self.__set_velocity(
            #     self.offset_rotation, self.angular_speed, pitch
            # )
            self._twist.angular.z = self.angular_speed * (abs(pitch) / pitch)  # Constant rotation

        self.__new_cmd = True
        self.get_logger().debug(f"Aruco pose. Y_VEL:{aruco_pose.position.x}, Z_VEL:{aruco_pose.position.y}, Distance:{aruco_pose.position.z}, Rotatation{pitch}")

    ########################### Velocity #######################################

    def __set_forward(self, __speed: float, __dist: float, __distance: float = 0.0):
        __speed /= 2
        if __dist < 1:
            __speed / (__dist**2)
        elif __dist < 2:
            __speed * __dist
        else:
            __speed * 2
        return __speed

    def __set_velocity(self, __offset: float, __speed: float, __dist: float):
        __speed /= 2
        if __dist < 1:
            __speed * (__dist**2) + 0.25
        elif __dist < 2:
            __speed * abs(__dist)
        else:
            __speed *= 2
        return __speed * (abs(__dist) / (__dist))

    def __send_cmd(self):

        self.get_logger().debug(f"Drone speed. Y_VEL:{self._twist.linear.y} X_VEL:{self._twist.linear.x} Z_VEL:{self._twist.linear.z} ROT_VEL:{self._twist.angular.z}")

        if self.sim:
            self._twist.linear.y *= -1
            self._twist.linear.z *= -1
            self._twist.angular.z *= -1
        else:
            # self._twist.linear.x *= -1 # Przód/Tył
            # self._twist.linear.y *= -1 # Lewo/Prawo
            self._twist.linear.z *= -1  # Góra/Dół
            # self._twist.angular.z *= -1 # Obrót
            pass
        if self.sending_method == "ros_service":
            req = TelloAction.Request()
            req.cmd = f"rc {int(self._twist.linear.y)} {int(self._twist.linear.x)} {int(self._twist.linear.z)} {int(self._twist.angular.z)}"
            self._call_service.call_async(req)
            self.get_logger().debug(f"{req.cmd}")

        self._pub_cmd_vel.publish(self._twist)
        self.get_logger().debug(f"{self._twist}")


# class TelloARL_PID(Node):
#     def __init__(self):
#         super().__init__("tello_arl")

#         self.declare_parameters(
#             namespace="",
#             parameters=[
#                 ("cmd_vel_topic", "/cmd_vel"),
#                 ("aruco_topic", "/aruco_poses"),
#                 ("flight_data_topic", "/flight_data"),
#                 ("log_level", 10),
#                 ("speed_horizontal", 20.0),
#                 ("speed_vertical", 15.0),
#                 ("speed_frontal", 20.0),
#                 ("speed_angular", 30.0),
#                 ("distance", 10.0),
#                 ("frequency", 1.0),
#                 ("velocity_send_method", "ros_service"),
#                 ("service_name", "/tello_action"),
#                 ("simulation", False),
#             ],
#         )
#         self.__init_variables()
#         self.sim = self.get_parameter("simulation").get_parameter_value().bool_value

#         self.distance = self.get_parameter("distance").get_parameter_value().double_value
#         self._logger.set_level(self.get_parameter("log_level").get_parameter_value().integer_value)
#         self.sending_method = self.get_parameter("velocity_send_method").get_parameter_value().string_value
#         self._sub_flight_data = self.create_subscription(
#             FlightData,
#             self.get_parameter("flight_data_topic").get_parameter_value().string_value,
#             self.flight_data_callback,
#             10,
#         )
#         self._sub_aruco_pose = self.create_subscription(
#             PoseArray,
#             self.get_parameter("aruco_topic").get_parameter_value().string_value,
#             self.aruco_pose_callback,
#             10,
#         )
#         self._pub_cmd_vel = self.create_publisher(
#             Twist,
#             self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
#             10,
#         )
#         self._call_service = self.create_client(
#             TelloAction,
#             self.get_parameter("service_name").get_parameter_value().string_value,
#         )
#         while not self._call_service.wait_for_service(timeout_sec=2.0):
#             self.get_logger().error("service not available, waiting again...")
#         self._response = self.create_subscription(TelloResponse, "tello_response", self.response_callback, 10)
#         self.waiting_response = False
#         self.create_timer(
#             1.0 / self.get_parameter("frequency").get_parameter_value().double_value,
#             self.timer_callback,
#         )
#         self._call_service.call_async(TelloAction.Request(cmd="takeoff"))
#         self._init_pid()
#         self.get_logger().info("TelloARL node has been started")

#     def _init_pid(self):

#         speed_horizontal = self.get_parameter("speed_horizontal").get_parameter_value().double_value
#         speed_vertical = self.get_parameter("speed_vertical").get_parameter_value().double_value
#         speed_frontal = self.get_parameter("speed_frontal").get_parameter_value().double_value
#         angular_speed = self.get_parameter("speed_angular").get_parameter_value().double_value
#         frequency = self.get_parameter("frequency").get_parameter_value().double_value

#         self.pid_x = PID(1, 0.1, 0.05, output_limits=(-speed_horizontal, speed_horizontal))
#         self.pid_y = PID(1, 0.1, 0.05, output_limits=(-speed_frontal, speed_frontal))
#         self.pid_z = PID(1, 0.1, 0.05, output_limits=(-speed_horizontal, speed_horizontal))
#         self.pid_r = PID(1, 0.1, 0.05, output_limits=(-angular_speed, angular_speed))

#         log = f"PID parameters:speed_horizontal: {speed_horizontal}, speed_vertical: {speed_vertical}, speed_frontal: {speed_frontal}, angular_speed: {angular_speed}, frequency: {frequency}"
#         self.get_logger().info(log)

#     def __init_variables(self):
#         self._flight_data = FlightData()
#         self._twist = Twist()
#         self._image = Image()
#         aruco_pose = Pose()
#         self.__new_cmd = None
#         self.last_direction = None
#         pitch = None

#     def __del__(self):
#         self.get_logger().info(f"Battery: {self._flight_data.bat}, Height: {self._flight_data.h}")
#         self._pub_cmd_vel.publish(Twist())
#         req = TelloAction.Request()
#         req.cmd = f"rc 0 0 0 0"
#         self._call_service.call_async(req)
#         self._call_service.call_async(TelloAction.Request(cmd="land"))
#         self.get_logger().info("TelloARL node has been stopped")

#     ########################### Callbacks #######################################

#     def response_callback(self, msg: TelloResponse):
#         self.get_logger().info(f"Response: {msg.rc}")

#     def flight_data_callback(self, msg: FlightData):
#         self.get_logger().debug("Flight data callback")
#         self._flight_data = msg

#     def aruco_pose_callback(self, msg):
#         aruco_pose = msg.poses[0]

#         _, pitch, _ = transform.Rotation.from_quat(
#             [
#                 aruco_pose.orientation.x,
#                 aruco_pose.orientation.y,
#                 aruco_pose.orientation.z,
#                 aruco_pose.orientation.w,
#             ]
#         ).as_euler("xyz")

#         self._twist.linear.x = self.pid_x(aruco_pose.position.z - self.distance)
#         self.get_logger().info(f"Distance: {aruco_pose.position.z - self.distance}")
#         self._twist.linear.y = self.pid_y(aruco_pose.position.x)
#         self._twist.linear.z = self.pid_z(aruco_pose.position.y)
#         self._twist.angular.z = self.pid_r(pitch)

#         self.__new_cmd = True

#     def timer_callback(self):
#         self.get_logger().debug(f"Timer callback. Battery: {self._flight_data.bat}, Height: {self._flight_data.h}")

#         if self.__new_cmd:
#             self.__send_cmd()
#             self.__new_cmd = False
#         else:
#             self.get_logger().debug("No new command")
#             self._twist = Twist()
#             self.__send_cmd()

#     def __send_cmd(self):
#         self.get_logger().debug(f"Aruco pose. Y_VEL:{aruco_pose.position.x}, Z_VEL:{aruco_pose.position.y}, Distance:{aruco_pose.position.z}, Rotatation{pitch}")
#         self.get_logger().debug(f"Drone speed. Y_VEL:{self._twist.linear.y} X_VEL:{self._twist.linear.x} Z_VEL:{self._twist.linear.z} ROT_VEL:{self._twist.angular.z}")

#         if self.sending_method == "ros_service":
#             req = TelloAction.Request()
#             req.cmd = f"rc {int(self._twist.linear.y)} {int(self._twist.linear.x)} {int(self._twist.linear.z)} {int(self._twist.angular.z)}"
#             self._pub_cmd_vel.publish(self._twist)

#             self._call_service.call_async(req)
#             self.get_logger().debug(f"{req.cmd}")

#         self._pub_cmd_vel.publish(self._twist)
#         self.get_logger().debug(f"{self._twist}")
