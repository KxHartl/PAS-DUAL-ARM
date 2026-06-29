#!/usr/bin/env python3
"""Aruco DICT_4X4_50 detector for the seminar task.

The PAL aruco_ros stack bundles its own marker library whose dictionaries
(ARUCO_MIP_36h12, ARTAG, AprilTag...) do NOT include the OpenCV DICT_4X4_50
chosen and documented for this assignment, so this node detects the marker with
cv2.aruco instead. It subscribes to the simulated RGB camera, estimates the
marker pose with solvePnP and publishes both a PoseStamped (in the camera
optical frame) and a TF (camera optical frame -> marker frame), which is what
the rest of the pipeline consumes.
"""
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def image_to_gray(msg):
    """sensor_msgs/Image -> mono8 numpy array without cv_bridge (the Humble
    cv_bridge binary is built against numpy 1.x and segfaults under numpy 2)."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = data.reshape(msg.height, msg.width, 3)
        code = cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(img, code)
    if msg.encoding == 'mono8':
        return data.reshape(msg.height, msg.width)
    raise ValueError(f'unsupported encoding {msg.encoding}')


def rvec_to_quat(rvec):
    """Rodrigues rotation vector -> quaternion (x, y, z, w)."""
    rot, _ = cv2.Rodrigues(rvec)
    # Shepperd's method via trace
    t = np.trace(rot)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.225)  # 0.75 * 0.3 m box face
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('marker_frame', 'aruco_marker_frame')

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_frame = self.get_parameter('marker_frame').value

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # Sub-pixel corner refinement gives markedly more stable corners (and
        # therefore a steadier solvePnP pose) than the default contour corners.
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        self.camera_matrix = None
        self.dist_coeffs = None

        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_single/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self.info_cb, 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_cb, 10)
        self.get_logger().info(
            f'Looking for DICT_4X4_50 marker id={self.marker_id} '
            f'size={self.marker_size} m')

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return
        gray = image_to_gray(msg)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return
        for corner, marker_id in zip(corners, ids.ravel()):
            if int(marker_id) != int(self.marker_id):
                continue
            half = self.marker_size / 2.0
            obj_pts = np.array([[-half, half, 0], [half, half, 0],
                                [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, corner.reshape(4, 2).astype(np.float64),
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            qx, qy, qz, qw = rvec_to_quat(rvec)

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(tvec[0])
            pose.pose.position.y = float(tvec[1])
            pose.pose.position.z = float(tvec[2])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pose_pub.publish(pose)

            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.marker_frame
            tf.transform.translation.x = float(tvec[0])
            tf.transform.translation.y = float(tvec[1])
            tf.transform.translation.z = float(tvec[2])
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
