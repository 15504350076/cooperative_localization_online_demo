#!/usr/bin/env python3
"""Pure geometry tests for the known-landmark visual localizer."""

import importlib.util
import math
from pathlib import Path
import unittest

import cv2
import numpy as np


SCRIPT = (
    Path(__file__).parents[1] / "scripts" / "colored_landmark_localizer.py"
)
SPEC = importlib.util.spec_from_file_location("colored_localizer", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TestColouredLandmarkLocalizer(unittest.TestCase):

    def test_colour_centres_and_ambiguity_rejection(self):
        image = np.zeros((100, 120, 3), dtype=np.uint8)
        colours = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0))
        expected = ((20, 20), (90, 20), (90, 75), (20, 75))
        for colour, (x, y) in zip(colours, expected):
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), colour, -1)
        centres = MODULE.extract_colour_centres(image, 25.0)
        self.assertIsNotNone(centres)
        np.testing.assert_allclose(centres, expected, atol=0.1)

        image[65:76, 45:56, 0] = 255
        self.assertIsNone(MODULE.extract_colour_centres(image, 25.0))

    def test_metric_pose_and_extrinsic_direction(self):
        landmark = np.asarray([
            [11.82, 3.20, 2.00],
            [11.82, 1.80, 2.00],
            [11.82, 1.80, 0.40],
            [11.82, 3.20, 0.40],
        ], dtype=np.float64)
        camera_matrix = np.asarray([
            [277.128, 0.0, 160.0],
            [0.0, 277.128, 120.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        base_position = np.asarray([2.0, 0.15, 0.25])
        yaw = 0.20
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotation_enu_base = np.asarray([
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ])
        extrinsic = np.asarray([0.41, 0.0, 0.10])
        rotation_enu_camera = (
            rotation_enu_base @ MODULE.R_BASE_CAMERA_OPTICAL
        )
        position_enu_camera = base_position + rotation_enu_base @ extrinsic
        rotation_camera_enu = rotation_enu_camera.T
        translation_camera_enu = -rotation_camera_enu @ position_enu_camera
        rotation_vector, _ = cv2.Rodrigues(rotation_camera_enu)
        image_points, _ = cv2.projectPoints(
            landmark, rotation_vector, translation_camera_enu,
            camera_matrix, np.zeros(5),
        )
        estimate = MODULE.estimate_base_pose(
            image_points.reshape(-1, 2), landmark, camera_matrix,
            np.zeros(5), extrinsic, yaw, 0.25, base_position,
        )
        self.assertIsNotNone(estimate)
        position, estimated_yaw, reprojection_rmse = estimate
        np.testing.assert_allclose(position, base_position, atol=1.0e-5)
        self.assertAlmostEqual(
            MODULE.wrap_angle(estimated_yaw - yaw), 0.0, delta=1.0e-6
        )
        self.assertLess(reprojection_rmse, 1.0e-5)

    def test_one_rasterized_centroid_outlier_does_not_move_the_vehicle(self):
        landmark = np.asarray([
            [11.82, 3.20, 2.00],
            [11.82, 1.80, 2.00],
            [11.82, 1.80, 0.40],
            [11.82, 3.20, 0.40],
        ], dtype=np.float64)
        camera_matrix = np.asarray([
            [277.128, 0.0, 160.0],
            [0.0, 277.128, 120.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        base_position = np.asarray([0.0, 0.0, 0.25])
        extrinsic = np.asarray([0.41, 0.0, 0.10])
        image_points, _ = MODULE._project_planar_pose(
            np.asarray([0.0, 0.0, 0.0]), landmark, camera_matrix,
            np.zeros(5), extrinsic, 0.25,
        )
        image_points[3] += np.asarray([0.3, -1.8])
        estimate = MODULE.estimate_base_pose(
            image_points, landmark, camera_matrix, np.zeros(5), extrinsic,
            0.0, 0.25, base_position,
        )
        self.assertIsNotNone(estimate)
        position, estimated_yaw, _ = estimate
        self.assertLess(np.linalg.norm(position[:2]), 0.25)
        self.assertLess(abs(estimated_yaw), 0.05)


if __name__ == "__main__":
    unittest.main()
