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
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, Point
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformBroadcaster

# costume import for getParams marker_size
from rcl_interfaces.srv import GetParameters
from ros2_aruco_interfaces.srv import UpdateParams


from rcl_interfaces.msg import SetParametersResult


class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Declare and read parameters
        self.declare_parameter("marker_size", 0.1435)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)
        #self.declare_parameter("filter_ids", [])

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', qos_profile_sensor_data)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', qos_profile_sensor_data)

        # self.request = GetParameters.Request()
        # self.request.names = ['marker_size']

        # Client and Service for getParams marker_size

        # self.srv = self.create_service(UpdateParams,
        #                                'update_params',
        #                                self.update_params_callback)
        #
        # self.client = self.create_client(GetParameters,
        #                                  '/aruco_node/get_parameters')

        self.add_on_set_parameters_callback(self.parameters_callback)

        # self.request = GetParameters.Request()
        # self.request.names = ['marker_size']

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def parameters_callback(self, params):
        for param in params:
            if param and param.name == "marker_size":
                self.marker_size = param.value
        return SetParametersResult(successful=True)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='mono8')
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        # timeimage = markers.header.stamp
        # timeimagesec = timeimage.sec
        # timeimagenano = timeimage.nanosec
        # if (len(str(timeimagenano)) != 9):
        #     timeimagenano = str(0)+str(timeimagenano)
        #
        # timeimagecomp = float(str(timeimagesec) + "." + str(timeimagenano))
        # timenode = self.get_clock().now().to_msg()
        # timenodesec = timenode.sec
        # timenodenano = timenode.nanosec
        # if (len(str(timenodenano)) != 9):
        #     timenodenano = str(0)+str(timenodenano)
        #
        # timenodecomp = float(str(timenodesec) + "." + str(timenodenano))
        #
        # diff = timenodecomp - timeimagecomp
        # self.get_logger().info("image %r" % str(img_msg.header))
        # self.get_logger().info("image %r" % str(timeimagenano))
        # self.get_logger().info("node %r" % str(timenodenano))
        # self.get_logger().info("diff %r" % str(diff))

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)

        if marker_ids is not None:
            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                      self.marker_size, self.intrinsic_mat,
                                                                      self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                   self.marker_size, self.intrinsic_mat,
                                                                   self.distortion)

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

                t = TransformStamped()
                # t.header.stamp = self.get_clock().now().to_msg()
                t.header.stamp = img_msg.header.stamp
                t.header.frame_id = 'camera'
                t.child_frame_id = "marker_" + str(marker_id[0])
                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.tf_broadcaster.sendTransform(t)

            for corner in corners[0]:
                p = Point()
                p.x = float(round(sum([row[0] for row in corner]) / 4))  # center x
                p.y = float(round(sum([row[1] for row in corner]) / 4)) # center y
                p.z = 0.0
                markers.pixel_centers.append(p)

            if not len(pose_array.poses) == 0:
                self.poses_pub.publish(pose_array)
                self.markers_pub.publish(markers)

    # def update_params_callback(self, request, response):
    #     if request.update:
    #         self.client.wait_for_service()
    #         future = self.client.call_async(self.request)
    #         future.add_done_callback(self.callback_global_param)
    #         response.done = True
    #         return response
    #     else:
    #         response.done = False
    #         return response
    #
    # def callback_global_param(self, future):
    #     try:
    #         result = future.result()
    #     except Exception as e:
    #         self.get_logger().warn("service call failed %r" % (e,))
    #     else:
    #         param = result.values[0]
    #         self.get_logger().info("Got global param: %s" % (param.double_value,))
    #         self.get_logger().info("Got global param2: %s" % (result.values,))


def main():
    rclpy.init()
    node = ArucoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
