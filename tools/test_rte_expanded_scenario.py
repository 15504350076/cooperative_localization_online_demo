"""针对低速编队场景和配对测距输入的回归检查。"""
import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

import evaluate_rte_expanded as sim
import evaluate_uwb_formation_sim as uwb
from evaluate_rte import EDGES, HEADINGS, rotate


class ExpandedScenarioTest(unittest.TestCase):
    def test_existing_uwb_library_geometry(self):
        lib = uwb.library(sim.ROOT / 'build_rte_verify/libzju_coop.so')
        result = uwb.solve(lib, (3.0, 4.0, 5.0))
        self.assertEqual(result.disposition, 0)
        np.testing.assert_allclose([result.x_m[1], result.y_m[1], result.x_m[2], result.y_m[2]],
                                   [3.0, 0.0, 0.0, 4.0], atol=1e-10)
        self.assertNotEqual(uwb.solve(lib, (1.0, 1.0, 3.0)).disposition, 0)

    def test_formation_speed_spacing_and_stationary_start(self):
        times = np.arange(0.0, sim.DURATION_S + 0.01, 0.1)
        world = np.array([[sim.world_position(n, t) for n in range(3)] for t in times])
        distances = np.array([np.linalg.norm(world[:, i] - world[:, j], axis=1) for i, j in EDGES])
        speeds = [np.linalg.norm(sim.motion(n, t)[1]) for t in times for n in range(3)]
        self.assertGreater(distances.min(), 5.0)
        self.assertLess(distances.max(), 30.0)
        self.assertLess(max(speeds), 3.0)
        self.assertGreater(np.ptp(world[:, :, 0]), 60.0)
        self.assertGreater(np.ptp(world[:, :, 1]), 60.0)
        for n in range(3):
            np.testing.assert_allclose(sim.motion(n, 0.0)[1], 0.0, atol=1e-12)
            np.testing.assert_allclose(sim.motion(n, sim.MOTION_START_S)[1], 0.0, atol=1e-12)
            for t in (35.0, 80.0, 200.0):
                dt = 1e-3
                numerical = (sim.world_position(n, t + dt) - sim.world_position(n, t - dt)) / (2 * dt)
                np.testing.assert_allclose(numerical, rotate(sim.motion(n, t)[1], HEADINGS[n]), atol=1e-6)

    def test_paired_runs_have_identical_ranges(self):
        old_duration, old_start = sim.DURATION_S, sim.MOTION_START_S
        try:
            sim.DURATION_S, sim.MOTION_START_S = 0.2, 0.0
            with tempfile.TemporaryDirectory() as directory:
                outputs = []
                for mode in ('odometry', 'odometry_small_error'):
                    path = Path(directory) / f'{mode}.csv'
                    sim.generate(path, path.with_suffix('.truth.csv'), mode, 0.05, 42)
                    with path.open(newline='') as stream:
                        outputs.append(list(csv.reader(stream))[1:])
                ranges = [[row for row in rows if row[0] == 'R'] for rows in outputs]
                self.assertEqual(ranges[0], ranges[1])
                odometry = [[row for row in rows if row[0] == 'O'] for rows in outputs]
                self.assertNotEqual(odometry[0], odometry[1])
        finally:
            sim.DURATION_S, sim.MOTION_START_S = old_duration, old_start


if __name__ == '__main__':
    unittest.main()
