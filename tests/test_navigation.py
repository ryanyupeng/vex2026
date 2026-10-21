"""Phase 5 integration tests against the shared deterministic differential plant."""
import contextlib
import io
import math
import unittest

from test_hbase import code, Robot, Result, Fault, Plant, Spec, Config


class NavigationConfig(Config):
    ROUTE_TIMEOUT_MS = 60000  # Individual navigation tests may exceed a match route.


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()

    def tearDown(self):
        self.output.__exit__(None, None, None)

    def robot(self, heading=0):
        hw = Plant()
        robot = Robot(hw, Spec, NavigationConfig)
        self.assertTrue(robot.prepare().ok())
        hw.theta = math.radians(heading)
        hw.imu.deg = heading
        robot.begin_autonomous(lambda: hw.enabled)
        robot.tracking.reset(0, 0, heading)
        return robot, hw

    def arrived(self, result, hw, x, y, heading=None):
        self.assertEqual(result.status, Result.ARRIVED, result.detail)
        self.assertLessEqual(math.hypot(hw.x - x, hw.y - y), 16)
        if heading is not None:
            self.assertLessEqual(abs(code["angle_error"](heading, hw.imu.deg)), 2.1)
        self.assertTrue(all(m.rpm == 0 for m in hw.drive))

    def test_absolute_goal_in_all_quadrants_and_final_heading(self):
        for x, y in ((300, 500), (-300, 500), (300, -500), (-300, -500)):
            with self.subTest(x=x, y=y):
                r, h = self.robot()
                self.arrived(r.go_to_pose(x, y, 90, 300, timeout_ms=15000), h, x, y, 90)

    def test_reverse_to_goal_and_heading_wrap(self):
        r, h = self.robot(350)
        self.arrived(r.go_to_pose(100, -500, 10, 300, reverse=True,
                                 timeout_ms=15000), h, 100, -500, 10)

    def test_repeated_absolute_goals_do_not_reset_pose(self):
        r, h = self.robot()
        self.arrived(r.go_to_pose(0, 400, 0, 250), h, 0, 400, 0)
        self.arrived(r.go_to_pose(300, 400, 90, 250), h, 300, 400, 90)
        self.arrived(r.go_to_pose(300, 0, 0, 250, reverse=True,
                                 timeout_ms=15000), h, 300, 0, 0)

    def test_nonzero_field_origin(self):
        r, h = self.robot(90)
        h.x, h.y = 1000, 2000
        r.tracking.reset(1000, 2000, 90)
        self.arrived(r.go_to_pose(1400, 2000, 90, 250), h, 1400, 2000, 90)

    def test_already_at_position_still_finishes_heading(self):
        r, h = self.robot()
        self.arrived(r.go_to_pose(0, 0, 90), h, 0, 0, 90)
        self.assertGreater(h.time, NavigationConfig.SETTLE_MS)

    def test_polyline_corners_and_duplicate_waypoints(self):
        r, h = self.robot()
        points = [(0, 0), (0, 400), (0, 400), (300, 400), (300, 700)]
        self.arrived(r.follow_path(points, 0, 250, timeout_ms=20000), h, 300, 700, 0)

    def test_closed_path_does_not_finish_at_start(self):
        r, h = self.robot()
        visited = []
        pause = h.pause
        def record():
            pause()
            visited.append((h.x, h.y))
        h.pause = record
        points = [(0, 300), (300, 300), (300, 0), (0, 0)]
        self.arrived(r.follow_path(points, 0, 250, timeout_ms=25000), h, 0, 0, 0)
        for x, y in points[:-1]:
            self.assertLess(min(math.hypot(px - x, py - y) for px, py in visited), 36)

    def test_variable_timing_asymmetry_and_lateral_disturbance(self):
        r, h = self.robot()
        h.left_efficiency = .85
        h.jitter = True
        h.disturb_at, h.disturb_mm = 500, 35
        self.arrived(r.go_to_pose(0, 800, 0, 300, timeout_ms=12000), h, 0, 800, 0)

    def test_bounded_rejoin_uses_x_sensor(self):
        r, h = self.robot()
        pause = h.pause
        def disturbance():
            if 400 <= h.time < 520:
                h.disturb_at, h.disturb_mm = h.time + 1, 20
            pause()
        h.pause = disturbance
        self.arrived(r.go_to_pose(0, 1000, 0, 300, timeout_ms=15000), h, 0, 1000, 0)

    def test_stalled_translation_turn_and_frozen_tracking(self):
        for blocked, frozen, x, y in ((True, False, 0, 500), (True, False, 500, 0),
                                     (False, True, 0, 500)):
            r, h = self.robot()
            h.blocked, h.freeze_tracking = blocked, frozen
            result = r.go_to_pose(x, y, 0, 300)
            self.assertEqual(result.status, Result.STALLED)
            self.assertTrue(all(m.rpm == 0 for m in h.drive))

    def test_cancel_timeout_disconnect_and_mechanism_fault(self):
        for fault in ("cancel", "timeout", "disconnect", "mechanism"):
            r, h = self.robot()
            timeout = 10000
            expected = Result.CANCELLED
            if fault == "cancel":
                h.cancel_at = 200
                r.lift.start(300, 80)
            elif fault == "timeout":
                timeout, expected = 100, Result.TIMED_OUT
            elif fault == "disconnect":
                h.x_sensor.connected = False
                expected = Result.SENSOR_FAULT
            else:
                r.lift.start(300, 80)
                h.lift[1].frozen = True
                expected = Result.MECHANISM_FAULT
            result = r.go_to_pose(0, 700, 0, 300, timeout_ms=timeout)
            self.assertEqual(result.status, expected)
            self.assertTrue(all(m.rpm == 0 for m in h.drive))
            self.assertTrue(all(not m.active for m in r.mechanisms))

    def test_invalid_input_stops_pending_actions(self):
        for points in ([], [(1,)], [(float("nan"), 0)], [(0, 0, 0)]):
            r, h = self.robot()
            r.lift.start(300, 80)
            with self.assertRaises(ValueError):
                r.follow_path(points)
            self.assertFalse(r.lift.active)
            self.assertTrue(all(m.rpm == 0 for m in h.drive))

    def test_navigation_requires_begin_autonomous(self):
        h = Plant()
        r = Robot(h, Spec, NavigationConfig)
        r.prepare()
        self.assertEqual(r.go_to_pose(0, 500).status, Result.NOT_READY)

    def test_geometry_keeps_order_at_crossing(self):
        path = code["NavigationPath"]((0, 0), [(0, 500), (500, 500), (0, 0)])
        path.sample((0, 0), 120, 35)
        self.assertEqual(path.index, 0)
        path.sample((0, 490), 120, 35)
        self.assertEqual(path.index, 1)
        path.sample((0, 0), 120, 35)
        self.assertEqual(path.index, 1)  # Can't jump to the close final point.

    def test_recovery_timeout_attempt_limit_and_hard_boundary(self):
        for policy in ("timeout", "attempts", "boundary"):
            with self.subTest(policy=policy):
                r, h = self.robot()
                class RecoveryConfig(NavigationConfig):
                    NAV_RECOVERY_TIMEOUT_MS = 20 if policy == "timeout" else 2000
                    NAV_RECOVERY_ATTEMPTS = 0 if policy == "attempts" else 2
                    NAV_MAX_CROSS_MM = 110 if policy == "boundary" else 300
                r.cfg = RecoveryConfig
                pause = h.pause
                def disturbance():
                    if 400 <= h.time < 560:
                        h.disturb_at, h.disturb_mm = h.time + 1, 20
                    pause()
                h.pause = disturbance
                result = r.go_to_pose(0, 1000, 0, 300)
                self.assertEqual(result.status, Result.PATH_ERROR)
                self.assertTrue(all(m.rpm == 0 for m in h.drive))

    def test_endpoint_disturbance_during_final_turn_is_rechecked(self):
        r, h = self.robot()
        pause = h.pause
        disturbed = [False]
        def disturb_final_turn():
            if not disturbed[0] and abs(h.y - 400) < 16 and h.imu.deg > 15:
                h.disturb_at, h.disturb_mm = h.time + 1, 35
                disturbed[0] = True
            pause()
        h.pause = disturb_final_turn
        result = r.go_to_pose(0, 400, 90, 250, timeout_ms=15000)
        self.assertTrue(disturbed[0])
        self.arrived(result, h, 0, 400, 90)

    def test_navigation_example(self):
        r, h = self.robot()
        code["run_navigation_example"](r)
        x, y = (300.0, 600.0)
        self.assertLess(math.hypot(h.x - x, h.y - y), 16)
        self.assertLess(abs(code["angle_error"](90.0, h.imu.deg)), 2.1)

    def test_navigation_honors_global_route_deadline(self):
        r, h = self.robot()
        r.route_deadline = h.time + 100
        r.lift.start(300, 80)
        result = r.go_to_pose(0, 1000, 0, 250, timeout_ms=10000)
        self.assertEqual(result.status, Result.TIMED_OUT)
        self.assertIn("time budget", result.detail)
        self.assertFalse(r.lift.active)


if __name__ == "__main__":
    unittest.main()
