#!/usr/bin/env python3
"""
Visual Servoing Node for Space Arm.

Implements Image-Based Visual Servoing (IBVS) that computes desired
end-effector velocity from image feature errors. The velocity command
is published for the MPC controller to track.

Visual servoing is particularly useful in space for:
- Docking with uncooperative targets
- Debris capture
- Satellite inspection and servicing
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped, PoseStamped
from cv_bridge import CvBridge


class VisualServoingNode(Node):
    """IBVS controller that drives arm based on image feature errors."""

    def __init__(self):
        super().__init__('visual_servoing')

        # Parameters
        self.declare_parameter('servoing_gain', 0.5)
        self.declare_parameter('feature_threshold', 5.0)  # pixels
        self.declare_parameter('max_velocity', 0.1)  # m/s
        self.declare_parameter('target_topic', '/space_arm/ee_camera/image_raw')

        self.gain = self.get_parameter('servoing_gain').value
        self.threshold = self.get_parameter('feature_threshold').value
        self.max_vel = self.get_parameter('max_velocity').value
        target_topic = self.get_parameter('target_topic').value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.target_features = None  # desired feature positions in image
        self.current_features = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, target_topic, self._image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/space_arm/ee_camera/camera_info',
            self._camera_info_callback, 10
        )

        # Publishers
        self.vel_pub = self.create_publisher(
            TwistStamped, '/space_arm/servo_velocity', 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, '/space_arm/visual_servo/debug_image', 10
        )

        self.get_logger().info('Visual servoing node initialized')

    def _camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsic matrix."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def _image_callback(self, msg: Image):
        """Process incoming image for visual servoing."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        # Detect features in the image
        features = self._detect_features(cv_image)

        if features is None or len(features) == 0:
            return

        self.current_features = features

        # If we have target features, compute servoing velocity
        if self.target_features is not None and self.camera_matrix is not None:
            velocity = self._compute_ibvs_velocity(
                self.current_features, self.target_features
            )
            self._publish_velocity(velocity, msg.header)

        # Publish debug visualization
        self._publish_debug_image(cv_image, features, msg.header)

    def _detect_features(self, image: np.ndarray):
        """
        Detect visual features in the image.

        Uses ArUco marker detection as a starting point -- common for
        space servicing targets. Can be extended with learned features.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ArUco marker detection
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # Return center points of detected markers
            centers = []
            for corner in corners:
                center = corner[0].mean(axis=0)
                centers.append(center)
            return np.array(centers)

        # Fallback: detect bright blobs (simulating target LEDs)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            centers = []
            for c in contours:
                M = cv2.moments(c)
                if M['m00'] > 10:  # minimum area filter
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']
                    centers.append([cx, cy])
            if centers:
                return np.array(centers)

        return None

    def _compute_ibvs_velocity(
        self, current: np.ndarray, desired: np.ndarray
    ) -> np.ndarray:
        """
        Compute camera velocity using Image-Based Visual Servoing.

        Uses the image Jacobian (interaction matrix) to map feature
        errors to camera velocity commands.
        """
        if len(current) != len(desired):
            self.get_logger().warn('Feature count mismatch')
            return np.zeros(6)

        # Feature error in pixels
        error = (current - desired).flatten()

        if np.linalg.norm(error) < self.threshold:
            self.get_logger().debug('Target reached (within threshold)')
            return np.zeros(6)

        # Normalized image coordinates
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Build interaction matrix for point features
        # Assumes estimated depth Z for each feature
        Z_est = 1.0  # meters -- should come from depth estimation

        L_stack = []
        for pt in current:
            x = (pt[0] - cx) / fx
            y = (pt[1] - cy) / fy

            L = np.array([
                [-1/Z_est, 0, x/Z_est, x*y, -(1+x*x), y],
                [0, -1/Z_est, y/Z_est, 1+y*y, -x*y, -x],
            ])
            L_stack.append(L)

        L_full = np.vstack(L_stack)

        # Normalized error
        error_norm = np.zeros_like(error)
        for i in range(0, len(error), 2):
            error_norm[i] = error[i] / fx
            error_norm[i + 1] = error[i + 1] / fy

        # Pseudo-inverse control law: v = -lambda * L^+ * e
        L_pinv = np.linalg.pinv(L_full)
        velocity = -self.gain * L_pinv @ error_norm

        # Clamp velocity
        linear_vel = np.linalg.norm(velocity[:3])
        if linear_vel > self.max_vel:
            velocity[:3] *= self.max_vel / linear_vel

        return velocity

    def _publish_velocity(self, velocity: np.ndarray, header):
        """Publish velocity command."""
        msg = TwistStamped()
        msg.header = header
        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        msg.twist.angular.x = float(velocity[3])
        msg.twist.angular.y = float(velocity[4])
        msg.twist.angular.z = float(velocity[5])
        self.vel_pub.publish(msg)

    def _publish_debug_image(self, image, features, header):
        """Publish annotated debug image."""
        debug = image.copy()

        # Draw detected features
        if features is not None:
            for pt in features:
                cv2.circle(debug, (int(pt[0]), int(pt[1])), 8, (0, 255, 0), 2)

        # Draw target features
        if self.target_features is not None:
            for pt in self.target_features:
                cv2.circle(debug, (int(pt[0]), int(pt[1])), 8, (0, 0, 255), 2)
                cv2.drawMarker(
                    debug, (int(pt[0]), int(pt[1])),
                    (0, 0, 255), cv2.MARKER_CROSS, 12, 2
                )

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            msg.header = header
            self.debug_image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Debug image publish error: {e}')

    def set_target_from_current(self):
        """Capture current features as the servoing target."""
        if self.current_features is not None:
            self.target_features = self.current_features.copy()
            self.get_logger().info(
                f'Target set: {len(self.target_features)} features captured'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
