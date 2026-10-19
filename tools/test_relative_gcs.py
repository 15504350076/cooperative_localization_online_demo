"""协同定位监视界面的位姿有效性、超时和 HTTP 只读接口回归测试。"""

import json
import threading
import unittest
import urllib.request
from types import SimpleNamespace

from relative_gcs import RelativeState, create_server


def pose_message(*, frame="coop_ref_1_relative", valid=True, yaw_valid=False,
                 x=2.0, stamp=100):
    """模拟本工程 CooperativePose2DArray 的必要字段。"""
    vehicle = lambda node_id, px, py: SimpleNamespace(
        node_id=node_id, x_m=px, y_m=py, yaw_rad=0.4,
        position_valid=valid, yaw_valid=yaw_valid,
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame,
                               stamp=SimpleNamespace(sec=stamp, nanosec=0)),
        reference_node_id=1,
        vehicles=[vehicle(1, 0.0, 0.0), vehicle(2, x, 0.0),
                  vehicle(3, 0.6, 1.2)],
    )


class RelativeGcsTests(unittest.TestCase):
    def test_valid_invalid_hold_lost_offline_and_recovery(self):
        state = RelativeState()
        self.assertTrue(state.ingest(pose_message(), received_at=10.0))
        fresh = state.snapshot(now=10.1)
        self.assertEqual(fresh["frame_id"], "coop_ref_1_relative")
        self.assertEqual(fresh["nodes"][1]["status"], "VALID")
        self.assertFalse(fresh["nodes"][1]["yaw_valid"])
        self.assertEqual(fresh["nodes"][1]["x"], 2.0)

        self.assertTrue(state.ingest(pose_message(valid=False, x=999), received_at=10.2))
        held = state.snapshot(now=10.25)["nodes"][1]
        self.assertEqual(held["status"], "HOLD")
        self.assertEqual(held["x"], 2.0)
        self.assertEqual(held["last_valid_age_ms"], 250)
        self.assertEqual(state.snapshot(now=11.1)["nodes"][1]["status"], "LOST")
        offline = state.snapshot(now=12.3)["nodes"][1]
        self.assertEqual(offline["status"], "OFFLINE")
        self.assertIsNone(offline["x"])

        self.assertTrue(state.ingest(pose_message(x=3.5), received_at=13.0))
        self.assertEqual(state.snapshot(now=13.1)["nodes"][1]["x"], 3.5)
        self.assertEqual(state.snapshot(now=13.1)["nodes"][1]["status"], "VALID")

    def test_frame_switch_clears_old_coordinates_and_bad_frame_is_rejected(self):
        state = RelativeState()
        self.assertTrue(state.ingest(pose_message(), received_at=1.0))
        self.assertFalse(state.ingest(pose_message(frame="map"), received_at=1.1))
        self.assertEqual(state.snapshot(now=1.1)["messages"], 1)
        self.assertTrue(state.ingest(pose_message(frame="coop_ref_1_enu", valid=False),
                                     received_at=1.2))
        changed = state.snapshot(now=1.25)
        self.assertEqual(changed["frame_id"], "coop_ref_1_enu")
        self.assertIsNone(changed["nodes"][1]["x"])
        self.assertEqual(changed["nodes"][1]["status"], "WAITING")
        self.assertTrue(state.ingest(pose_message(frame="coop_ref_1_enu", yaw_valid=True),
                                     received_at=1.3))
        self.assertTrue(state.snapshot(now=1.4)["nodes"][1]["yaw_valid"])

    def test_http_snapshot_and_page(self):
        state = RelativeState(demo=True)
        state.ingest(pose_message(), received_at=1.0)
        server = create_server("127.0.0.1", 0, state)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            with urllib.request.urlopen(base + "/api/state") as response:
                payload = json.load(response)
                self.assertTrue(payload["demo"])
                self.assertEqual(payload["reference_node_id"], 1)
            with urllib.request.urlopen(base) as response:
                page = response.read().decode("utf-8")
                self.assertIn("浙大协同定位", page)
                self.assertIn("不是地图/北向坐标", page)
                self.assertNotIn("兜底", page)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
