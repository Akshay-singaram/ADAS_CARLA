#!/usr/bin/env python3
"""
Target Detector Node for Space Arm.

Detects and tracks objects of interest (e.g., satellite components,
docking ports, debris) in the end-effector camera feed. Publishes
detected target poses for the visual servoing pipeline.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class TargetDetectorNode(Node):
    """Detects and localizes targets using the end-effector camera."""

    def __init__(self):
        super().__init__('target_detector')

        self.declare_parameter('marker_size', 0.1)  # meters
        self.declare_parameter('detection_method', 'aruco')  # aruco | color | template

        self.marker_size = self.get_parameter('marker_size').value
        self.detection_method = self.get_parameter('detection_method').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/space_arm/ee_camera/image_raw',
            self._image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/space_arm/ee_camera/camera_info',
            self._camera_info_callback, 10
        )

        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/space_arm/detected_target_pose', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/space_arm/target_detector/annotated', 10
        )

        self.get_logger().info(
            f'Target detector initialized (method: {self.detection_method})'
        )

    def _camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def _image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        if self.detection_method == 'aruco':
            self._detect_aruco(cv_image, msg.header)
        elif self.detection_method == 'color':
            self._detect_color_target(cv_image, msg.header)

    def _detect_aruco(self, image: np.ndarray, header):
        """Detect ArUco markers and estimate their 6-DOF pose."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        annotated = image.copy()

        if ids is not None and self.camera_matrix is not None:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            # Estimate pose for each marker
            for i, marker_id in enumerate(ids.flatten()):
                # Define marker corners in 3D (marker frame)
                half = self.marker_size / 2
                obj_points = np.array([
                    [-half, half, 0],
                    [half, half, 0],
                    [half, -half, 0],
                    [-half, -half, 0],
                ], dtype=np.float64)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[i][0],
                    self.camera_matrix, self.dist_coeffs
                )

                if success:
                    cv2.drawFrameAxes(
                        annotated, self.camera_matrix, self.dist_coeffs,
                        rvec, tvec, self.marker_size * 0.5
                    )
                    self._publish_pose(rvec, tvec, header, marker_id)

        # Publish annotated image
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            ann_msg.header = header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().error(f'Annotated image error: {e}')

    def _detect_color_target(self, image: np.ndarray, header):
        """Detect bright colored targets (simulating LEDs or reflectors)."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect bright green targets
        lower = np.array([40, 100, 100])
        upper = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        annotated = image.copy()
        for c in contours:
            area = cv2.contourArea(c)
            if area > 50:
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(annotated, (cx, cy), 10, (0, 255, 0), 2)
                cv2.putText(
                    annotated, f'Target ({cx},{cy})',
                    (cx + 15, cy), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1
                )

        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            ann_msg.header = header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().error(f'Annotated image error: {e}')

    def _publish_pose(self, rvec, tvec, header, marker_id: int):
        """Convert OpenCV pose to ROS PoseStamped and publish."""
        from scipy.spatial.transform import Rotation

        msg = PoseStamped()
        msg.header = header
        msg.header.frame_id = 'camera_optical_frame'

        msg.pose.position.x = float(tvec[0])
        msg.pose.position.y = float(tvec[1])
        msg.pose.position.z = float(tvec[2])

        rot = Rotation.from_rotvec(rvec.flatten())
        q = rot.as_quat()  # [x, y, z, w]
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.target_pose_pub.publish(msg)
        self.get_logger().debug(
            f'Marker {marker_id} at [{tvec[0][0]:.3f}, {tvec[1][0]:.3f}, {tvec[2][0]:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
