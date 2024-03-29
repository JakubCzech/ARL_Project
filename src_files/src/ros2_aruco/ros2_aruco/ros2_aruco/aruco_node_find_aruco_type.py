#!/usr/bin/env python3

"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("marker_size", 0.0625),
                ("aruco_dictionary_id", "DICT_6X6_100"),
                ("image_topic", "/drone1/image_raw"),
                ("camera_info_topic", "/drone1/camera_info"),
                ("camera_frame", None),
                ("aruco_poses_topic", "/aruco_poses"),
                ("aruco_markers_topic", "/aruco_markers"),
            ],
        )
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        aruco_poses_topic = self.get_parameter("aruco_poses_topic").get_parameter_value().string_value
        aruco_markers_topic = self.get_parameter("aruco_markers_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)

        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, aruco_poses_topic, 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, aruco_markers_topic, 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()
        self.get_logger().info("Aruco detector started!")

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        # save the image for debugging
        cv2.imwrite("image.png", cv_image)
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame is None:
            self.get_logger().warn("Camera callback!")
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
            self.get_logger().error("No camera info has been received!")

        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp
        #  create loop with detect every type of marker
        markers_good = set()

        for dictionary_id in [
            cv2.aruco.DICT_4X4_50,
            cv2.aruco.DICT_4X4_100,
            cv2.aruco.DICT_4X4_250,
            cv2.aruco.DICT_4X4_1000,
            cv2.aruco.DICT_5X5_50,
            cv2.aruco.DICT_5X5_100,
            cv2.aruco.DICT_5X5_250,
            cv2.aruco.DICT_5X5_1000,
            cv2.aruco.DICT_6X6_50,
            cv2.aruco.DICT_6X6_100,
            cv2.aruco.DICT_6X6_250,
            cv2.aruco.DICT_6X6_1000,
            cv2.aruco.DICT_7X7_50,
            cv2.aruco.DICT_7X7_100,
            cv2.aruco.DICT_7X7_250,
            cv2.aruco.DICT_7X7_1000,
            cv2.aruco.DICT_ARUCO_ORIGINAL,
        ]:
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

            corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, aruco_dictionary, parameters=self.aruco_parameters)
            if marker_ids is not None:
                #  add the marker ids to the set
                markers_good.add(dictionary_id)
                if cv2.__version__ > "4.0.0":
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
                else:
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
                for i, marker_id in enumerate(marker_ids):
                    pose = Pose()
                    pose.position.x = tvecs[i][0][0]
                    pose.position.y = tvecs[i][0][1]
                    pose.position.z = tvecs[i][0][2]

                    rot_matrix = np.eye(4)
                    rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                    quat = transformations.quaternion_from_matrix(rot_matrix)

                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]

                    pose_array.poses.append(pose)
                    markers.poses.append(pose)
                    markers.marker_ids.append(marker_id[0])

                self.poses_pub.publish(pose_array)
                self.markers_pub.publish(markers)
        self.get_logger().info(f"Markers found: {markers_good}")


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
