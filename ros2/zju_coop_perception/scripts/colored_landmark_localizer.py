#!/usr/bin/env python3
"""Metric monocular localization from one known four-colour landmark board."""

import math

import cv2
import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


COLOUR_NAMES = ("red", "green", "blue", "yellow")
R_BASE_CAMERA_OPTICAL = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)
UNKNOWN_VARIANCE = 1.0e6


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_ns(stamp):
    if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _colour_masks(rgb_image):
    red = rgb_image[:, :, 0]
    green = rgb_image[:, :, 1]
    blue = rgb_image[:, :, 2]
    return (
        (red > 160) & (green < 100) & (blue < 100),
        (green > 160) & (red < 100) & (blue < 100),
        (blue > 160) & (red < 100) & (green < 100),
        (red > 160) & (green > 160) & (blue < 100),
    )


def extract_colour_centres(rgb_image, minimum_area_px):
    """Return red/green/blue/yellow centroids or None for ambiguous input."""
    centres = []
    for mask in _colour_masks(rgb_image):
        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if not contours or cv2.contourArea(contours[0]) < minimum_area_px:
            return None
        largest_area = cv2.contourArea(contours[0])
        if (
            len(contours) > 1
            and cv2.contourArea(contours[1]) > 0.5 * largest_area
        ):
            return None
        moments = cv2.moments(contours[0])
        if moments["m00"] <= 0.0:
            return None
        centres.append(
            [moments["m10"] / moments["m00"],
             moments["m01"] / moments["m00"]]
        )
    return np.asarray(centres, dtype=np.float64)


def _project_planar_pose(state, landmark_points_enu, camera_matrix,
                         distortion, base_to_camera_translation,
                         base_z_m):
    x_m, y_m, yaw_rad = state
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    rotation_enu_base = np.asarray([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    rotation_enu_camera = rotation_enu_base @ R_BASE_CAMERA_OPTICAL
    position_enu_camera = (
        np.asarray([x_m, y_m, base_z_m])
        + rotation_enu_base @ base_to_camera_translation
    )
    rotation_camera_enu = rotation_enu_camera.T
    translation_camera_enu = -rotation_camera_enu @ position_enu_camera
    rotation_vector, _ = cv2.Rodrigues(rotation_camera_enu)
    projected, _ = cv2.projectPoints(
        landmark_points_enu, rotation_vector, translation_camera_enu,
        camera_matrix, distortion,
    )
    depths = (
        rotation_camera_enu @ landmark_points_enu.T
        + translation_camera_enu.reshape(3, 1)
    )[2, :]
    return projected.reshape(-1, 2), depths


def estimate_base_pose(image_points, landmark_points_enu, camera_matrix,
                       distortion, base_to_camera_translation,
                       expected_yaw_rad, expected_base_z_m,
                       expected_position_enu=None):
    """Estimate planar x/y/yaw from a known metric landmark."""
    # ponytail: the current cars are planar. A full 6DoF PnP frontend belongs
    # here only when non-planar vehicle motion is introduced.
    initial_position = (
        np.asarray(expected_position_enu, dtype=np.float64)
        if expected_position_enu is not None
        else np.asarray([0.0, 0.0, expected_base_z_m])
    )
    initial_state = np.asarray([
        initial_position[0], initial_position[1], expected_yaw_rad
    ], dtype=np.float64)
    step_sizes = (1.0e-3, 1.0e-3, 1.0e-4)
    candidates = []
    # A rasterized target edge can move one centroid by a couple of pixels.
    # Solve all four leave-one-out triples as well as the full set, then rank
    # them by the three best reprojection residuals. This is the smallest
    # robust estimator that fits the fixed four-target contract.
    subsets = [range(4)] + [
        [index for index in range(4) if index != omitted]
        for omitted in range(4)
    ]
    for subset in subsets:
        state = initial_state.copy()
        selected_landmarks = landmark_points_enu[subset]
        selected_pixels = image_points[subset]
        try:
            for _ in range(12):
                projected, _ = _project_planar_pose(
                    state, selected_landmarks, camera_matrix, distortion,
                    base_to_camera_translation, expected_base_z_m,
                )
                residual = (projected - selected_pixels).reshape(-1)
                jacobian = np.empty((residual.size, 3), dtype=np.float64)
                for column, step_size in enumerate(step_sizes):
                    displaced = state.copy()
                    displaced[column] += step_size
                    shifted, _ = _project_planar_pose(
                        displaced, selected_landmarks, camera_matrix,
                        distortion, base_to_camera_translation,
                        expected_base_z_m,
                    )
                    jacobian[:, column] = (
                        (shifted - projected) / step_size
                    ).reshape(-1)
                increment = np.linalg.lstsq(
                    jacobian, -residual, rcond=None
                )[0]
                increment = np.clip(
                    increment, [-0.5, -0.5, -0.10], [0.5, 0.5, 0.10]
                )
                state += increment
                state[2] = wrap_angle(state[2])
                if np.linalg.norm(increment) < 1.0e-6:
                    break
        except (cv2.error, np.linalg.LinAlgError):
            continue

        projected, depths = _project_planar_pose(
            state, landmark_points_enu, camera_matrix, distortion,
            base_to_camera_translation, expected_base_z_m,
        )
        if not np.all(np.isfinite(state)) or np.any(depths <= 0.1):
            continue
        point_errors = np.linalg.norm(projected - image_points, axis=1)
        robust_rmse = float(np.sqrt(np.mean(
            np.sort(point_errors * point_errors)[:3]
        )))
        prior_distance = float(np.linalg.norm(
            state[:2] - initial_state[:2]
        ))
        prior_yaw_error = abs(wrap_angle(state[2] - initial_state[2]))
        # The configured initial pose is known and subsequent frames arrive
        # at 10 Hz. Prefer the locally continuous solution when coplanar
        # candidates have nearly identical pixel residuals.
        score = robust_rmse + prior_distance + prior_yaw_error
        candidates.append((score, state, projected))

    if not candidates:
        return None
    _, state, projected = min(candidates, key=lambda candidate: candidate[0])
    residual = projected - image_points
    reprojection_rmse = float(np.sqrt(np.mean(np.sum(
        residual * residual, axis=1
    ))))
    position = np.asarray([state[0], state[1], expected_base_z_m])
    return position, float(state[2]), reprojection_rmse


class ColouredLandmarkLocalizer(Node):

    def __init__(self):
        super().__init__("zju_colored_landmark_localizer")
        self.output_frame_id = self.declare_parameter(
            "output_frame_id", "common_enu"
        ).value
        self.child_frame_id = self.declare_parameter(
            "child_frame_id", "vehicle/base_link"
        ).value
        self.expected_yaw_rad = float(self.declare_parameter(
            "expected_yaw_rad", 0.0
        ).value)
        self.expected_base_z_m = float(self.declare_parameter(
            "expected_base_z_m", 0.25
        ).value)
        initial_position_values = self.declare_parameter(
            "initial_position_enu_m", [0.0, 0.0, 0.25]
        ).value
        self.minimum_blob_area_px = float(self.declare_parameter(
            "minimum_blob_area_px", 12.0
        ).value)
        self.max_reprojection_error_px = float(self.declare_parameter(
            "max_reprojection_error_px", 2.5
        ).value)
        self.max_yaw_error_rad = float(self.declare_parameter(
            "max_yaw_error_rad", 0.8
        ).value)
        self.max_height_error_m = float(self.declare_parameter(
            "max_height_error_m", 0.75
        ).value)
        self.max_position_jump_m = float(self.declare_parameter(
            "max_position_jump_m", 0.75
        ).value)
        self.max_initial_position_error_m = float(self.declare_parameter(
            "max_initial_position_error_m", 3.0
        ).value)
        self.max_yaw_jump_rad = float(self.declare_parameter(
            "max_yaw_jump_rad", 0.15
        ).value)
        self.position_variance_m2 = float(self.declare_parameter(
            "position_variance_m2", 0.04
        ).value)
        self.yaw_variance_rad2 = float(self.declare_parameter(
            "yaw_variance_rad2", 0.01
        ).value)
        landmark_values = self.declare_parameter(
            "landmark_points_enu_m",
            [
                11.82, 3.20, 2.00,
                11.82, 1.80, 2.00,
                11.82, 1.80, 0.40,
                11.82, 3.20, 0.40,
            ],
        ).value
        extrinsic_values = self.declare_parameter(
            "base_to_camera_translation_flu_m", [0.41, 0.0, 0.10]
        ).value
        self._validate_parameters(
            landmark_values, extrinsic_values, initial_position_values
        )
        self.landmark_points_enu = np.asarray(
            landmark_values, dtype=np.float64
        ).reshape(4, 3)
        self.base_to_camera_translation = np.asarray(
            extrinsic_values, dtype=np.float64
        )
        self.initial_position = np.asarray(
            initial_position_values, dtype=np.float64
        )

        self.camera_matrix = None
        self.distortion = np.zeros(5, dtype=np.float64)
        self.last_stamp_ns = 0
        self.last_position = None
        self.last_yaw = None
        self.first_diagnostic_logged = False
        self.publisher = self.create_publisher(
            Odometry, "visual_odometry", 5
        )
        self.info_subscription = self.create_subscription(
            CameraInfo, "camera_info", self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            Image, "camera_image", self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "known-landmark visual localization ready; output is not fused"
        )

    def _validate_parameters(self, landmark_values, extrinsic_values,
                             initial_position_values):
        def finite_positive(value):
            return math.isfinite(value) and value > 0.0

        if not self.output_frame_id or not self.child_frame_id:
            raise ValueError("output_frame_id and child_frame_id are required")
        if len(landmark_values) != 12 or not all(
                math.isfinite(value) for value in landmark_values):
            raise ValueError(
                "landmark_points_enu_m must contain 12 finite values"
            )
        if len(extrinsic_values) != 3 or not all(
                math.isfinite(value) for value in extrinsic_values):
            raise ValueError(
                "base_to_camera_translation_flu_m must contain 3 finite values"
            )
        if len(initial_position_values) != 3 or not all(
                math.isfinite(value) for value in initial_position_values):
            raise ValueError(
                "initial_position_enu_m must contain 3 finite values"
            )
        if not all(finite_positive(value) for value in (
                self.minimum_blob_area_px, self.max_reprojection_error_px,
                self.max_yaw_error_rad, self.max_height_error_m,
                self.max_position_jump_m, self.max_initial_position_error_m,
                self.max_yaw_jump_rad,
                self.position_variance_m2,
                self.yaw_variance_rad2)):
            raise ValueError("visual localization thresholds must be positive")

    def _on_camera_info(self, message):
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if (matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0 or
                not np.all(np.isfinite(matrix))):
            return
        self.camera_matrix = matrix
        distortion = np.asarray(message.d, dtype=np.float64)
        self.distortion = (
            distortion
            if distortion.size >= 4 and np.all(np.isfinite(distortion))
            else np.zeros(5, dtype=np.float64)
        )

    @staticmethod
    def _rgb_image(message):
        if message.height <= 0 or message.width <= 0:
            return None
        channels = 3
        if message.step < message.width * channels:
            return None
        expected = message.height * message.step
        if len(message.data) < expected:
            return None
        rows = np.frombuffer(message.data, dtype=np.uint8, count=expected)
        rows = rows.reshape(message.height, message.step)
        image = rows[:, :message.width * channels].reshape(
            message.height, message.width, channels
        )
        if message.encoding.lower() == "rgb8":
            return image
        if message.encoding.lower() == "bgr8":
            return image[:, :, ::-1]
        return None

    def _on_image(self, message):
        current_stamp_ns = stamp_ns(message.header.stamp)
        if (current_stamp_ns <= self.last_stamp_ns or
                self.camera_matrix is None):
            return
        rgb_image = self._rgb_image(message)
        if rgb_image is None:
            return
        image_points = extract_colour_centres(
            rgb_image, self.minimum_blob_area_px
        )
        if image_points is None:
            return
        estimate = estimate_base_pose(
            image_points, self.landmark_points_enu, self.camera_matrix,
            self.distortion, self.base_to_camera_translation,
            self.last_yaw
            if self.last_yaw is not None else self.expected_yaw_rad,
            self.expected_base_z_m,
            self.last_position
            if self.last_position is not None else self.initial_position,
        )
        if estimate is None:
            return
        position, yaw, reprojection_rmse = estimate
        if (reprojection_rmse > self.max_reprojection_error_px or
                abs(float(position[2]) - self.expected_base_z_m) >
                self.max_height_error_m or
                abs(wrap_angle(yaw - self.expected_yaw_rad)) >
                self.max_yaw_error_rad):
            return
        previous_position = self.last_position
        position_gate_m = self.max_position_jump_m
        if previous_position is None:
            previous_position = self.initial_position
            position_gate_m = self.max_initial_position_error_m
        previous_yaw = (
            self.last_yaw
            if self.last_yaw is not None else self.expected_yaw_rad
        )
        if (np.linalg.norm(position[:2] - previous_position[:2]) >
                position_gate_m or
                abs(wrap_angle(yaw - previous_yaw)) >
                self.max_yaw_jump_rad):
            return

        if not self.first_diagnostic_logged:
            self.get_logger().info(
                "first landmark solution: "
                f"pixels={image_points.tolist()}, "
                f"position={position.tolist()}, yaw={yaw:.3f}"
            )
            self.first_diagnostic_logged = True

        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.output_frame_id
        output.child_frame_id = self.child_frame_id
        output.pose.pose.position.x = float(position[0])
        output.pose.pose.position.y = float(position[1])
        output.pose.pose.position.z = float(position[2])
        output.pose.pose.orientation.z = math.sin(0.5 * yaw)
        output.pose.pose.orientation.w = math.cos(0.5 * yaw)
        output.pose.covariance[0] = self.position_variance_m2
        output.pose.covariance[7] = self.position_variance_m2
        output.pose.covariance[14] = 1.0
        output.pose.covariance[21] = 1.0
        output.pose.covariance[28] = 1.0
        output.pose.covariance[35] = self.yaw_variance_rad2
        for index in (0, 7, 14, 21, 28, 35):
            output.twist.covariance[index] = UNKNOWN_VARIANCE
        self.publisher.publish(output)
        self.last_stamp_ns = current_stamp_ns
        self.last_position = position
        self.last_yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ColouredLandmarkLocalizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can race a subscription take with context shutdown. Keep
        # genuine runtime conversion errors visible while allowing a clean
        # launch-managed stop.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
