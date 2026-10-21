"""Host-only tests: deterministic simple plant, NOT a VEX firmware/field emulator."""
import contextlib
import io
import math
from pathlib import Path
import runpy
import sys
import types
import unittest


vex = types.ModuleType("vex")
for name in ("MSEC", "DEGREES", "RPM", "PERCENT", "HOLD", "BRAKE", "FORWARD", "REVERSE"):
    setattr(vex, name, name)
sys.modules["vex"] = vex
code = runpy.run_path(str(Path(__file__).resolve().parents[1] / "hbase.v5python"))
SpecBase, Config = code["RobotSpec"], code["ControlConfig"]
Robot, Result, Fault = code["Robot"], code["Result"], code["ActionFault"]


class Spec(SpecBase):
    TRACK_X_DIAMETER_MM = TRACK_Y_DIAMETER_MM = 50.0
    TRACK_X_ROTATION_MM_PER_RAD = 40.0
    TRACK_Y_ROTATION_MM_PER_RAD = -60.0
    DRIVE_WHEEL_DIAMETERS_MM = (100.0,) * 6
    DRIVE_MOTOR_TURNS_PER_WHEEL_TURN = (1.0,) * 6


class Motor:
    def __init__(self):
        self.rpm = self.deg = 0.0
        self.target = None
        self.connected = True
        self.frozen = False

    def installed(self):
        return self.connected

    def spin(self, direction, speed, units):
        assert units == "RPM"
        self.rpm = speed if direction == "FORWARD" else -speed

    def stop(self, mode):
        self.rpm = 0.0
        self.target = None

    def position(self, units):
        return self.deg

    def velocity(self, units):
        return self.rpm

    def set_position(self, value, units):
        self.deg = value

    def set_stopping(self, mode):
        pass

    def set_timeout(self, timeout, units):
        pass

    def spin_to_position(self, target, units, speed, speed_units, wait=False):
        self.target = target

    def advance(self, dt):
        if self.target is None or self.frozen:
            return
        change = code["clamp"](self.target - self.deg, -360 * dt, 360 * dt)
        self.deg += change
        self.rpm = change / dt / 6.0


class IMU(Motor):
    calibrating_forever = False

    def rotation(self, units):
        return self.deg

    def calibrate(self):
        pass

    def is_calibrating(self):
        return self.calibrating_forever


class Claw:
    def __init__(self):
        self.state = False

    def set(self, state):
        self.state = state


class Plant:
    """Differential base with lag and independent passive wheel measurements."""
    def __init__(self, spec=Spec):
        self.spec = spec
        self.drive = tuple(Motor() for _ in range(6))
        self.lift = (Motor(), Motor())
        self.wings = (Motor(), Motor())
        self.x_sensor, self.y_sensor, self.imu = Motor(), Motor(), IMU()
        self.claw = Claw()
        self.time = 0
        self.x = self.y = self.theta = 0.0
        self.left_speed = self.right_speed = 0.0
        self.left_efficiency = self.right_efficiency = 1.0
        self.enabled = True
        self.blocked = False
        self.freeze_tracking = False
        self.cancel_at = None
        self.disturb_at = None
        self.disturb_mm = 0.0
        self.step_ms = 20
        self.jitter = False

    def now(self):
        return self.time

    def show(self, message):
        pass

    def check_devices(self):
        if any(not d.installed() for d in self.drive + self.lift + self.wings +
               (self.x_sensor, self.y_sensor, self.imu)):
            raise Fault(Result.SENSOR_FAULT, "disconnected")
        if self.imu.is_calibrating():
            raise Fault(Result.SENSOR_FAULT, "calibrating")

    def tracking_degrees(self, mm, axis):
        diameter = getattr(self.spec, "TRACK_" + axis + "_DIAMETER_MM")
        ratio = getattr(self.spec, "TRACK_" + axis + "_SENSOR_TURNS_PER_WHEEL_TURN")
        sign = getattr(self.spec, "TRACK_" + axis + "_SIGN")
        return mm / (math.pi * diameter) * 360.0 * ratio / sign

    def pause(self):
        ms = self.step_ms + (10 if self.jitter and self.time % 40 == 0 else 0)
        dt = ms / 1000.0
        self.time += ms
        velocities = [m.rpm / ratio * math.pi * diameter / 60.0
                      for m, ratio, diameter in zip(self.drive,
                          self.spec.DRIVE_MOTOR_TURNS_PER_WHEEL_TURN,
                          self.spec.DRIVE_WHEEL_DIAMETERS_MM)]
        alpha = dt / (0.08 + dt)
        self.left_speed += alpha * (sum(velocities[:3]) / 3 * self.left_efficiency - self.left_speed)
        self.right_speed += alpha * (sum(velocities[3:]) / 3 * self.right_efficiency - self.right_speed)
        forward = 0 if self.blocked else (self.left_speed + self.right_speed) / 2 * dt
        delta = 0 if self.blocked else (self.left_speed - self.right_speed) / 300.0 * dt
        lateral = 0.0
        if self.disturb_at is not None and self.time >= self.disturb_at:
            lateral, self.disturb_at = self.disturb_mm, None
        mid = self.theta + delta / 2
        self.x += forward * math.sin(mid) + lateral * math.cos(mid)
        self.y += forward * math.cos(mid) - lateral * math.sin(mid)
        self.theta += delta
        self.imu.deg = math.degrees(self.theta) / self.spec.IMU_SIGN
        if not self.freeze_tracking:
            self.x_sensor.deg += self.tracking_degrees(lateral + delta * self.spec.TRACK_X_ROTATION_MM_PER_RAD, "X")
            self.y_sensor.deg += self.tracking_degrees(forward + delta * self.spec.TRACK_Y_ROTATION_MM_PER_RAD, "Y")
        for motor in self.lift + self.wings:
            motor.advance(dt)
        if self.cancel_at is not None and self.time >= self.cancel_at:
            self.enabled = False


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()

    def tearDown(self):
        self.output.__exit__(None, None, None)

    def robot(self, spec=Spec, config=Config):
        hw = Plant(spec)
        robot = Robot(hw, spec, config)
        self.assertTrue(robot.prepare().ok())
        robot.begin_autonomous(lambda: hw.enabled)
        return robot, hw

    def stopped(self, hw):
        self.assertTrue(all(m.rpm == 0 for m in hw.drive))

    def test_unknown_specs_refuse_readiness(self):
        robot = Robot(Plant(), SpecBase)
        self.assertEqual(robot.prepare().status, Result.NOT_READY)
        self.assertFalse(robot.ready)

    def test_signed_forward_and_reverse(self):
        for distance in (500, -500):
            with self.subTest(distance=distance):
                robot, hw = self.robot()
                result = robot.drive_distance(distance, 0, 400)
                self.assertEqual(result.status, Result.ARRIVED)
                self.assertLess(abs(hw.y - distance), 12)
                self.stopped(hw)

    def test_variable_dt_and_unequal_drive_response(self):
        robot, hw = self.robot()
        hw.jitter = True
        hw.left_efficiency = 0.85
        result = robot.drive_distance(650, 0, 450)
        self.assertEqual(result.status, Result.ARRIVED)
        self.assertLess(abs(hw.x), 16)
        self.assertLess(abs(hw.y - 650), 12)
        self.assertLess(abs(math.degrees(hw.theta)), 2.1)

    def test_lateral_tracking_correction_forward_and_reverse(self):
        for distance in (800, -800):
            with self.subTest(distance=distance):
                robot, hw = self.robot()
                hw.disturb_at, hw.disturb_mm = 400, 25
                result = robot.drive_distance(distance, 0, 400, timeout_ms=6500)
                self.assertEqual(result.status, Result.ARRIVED)
                self.assertLess(abs(hw.x), 15)
                self.assertLess(abs(robot.tracking.x - hw.x), 1)

    def test_turn_both_directions_and_wrap(self):
        for initial, target in ((0, 90), (0, -90), (350, 10), (10, 350)):
            with self.subTest(initial=initial, target=target):
                robot, hw = self.robot()
                hw.theta = math.radians(initial)
                hw.imu.deg = initial
                robot.tracking.reset(0, 0, initial)
                result = robot.turn_to_heading(target)
                self.assertEqual(result.status, Result.ARRIVED)
                self.assertLess(abs(code["angle_error"](target, math.degrees(hw.theta))), 2.1)
                self.assertLess(math.hypot(robot.tracking.x, robot.tracking.y), 0.01)
                self.stopped(hw)

    def test_stall_forward_reverse_and_turn(self):
        for action in ("forward", "reverse", "turn"):
            robot, hw = self.robot()
            hw.blocked = True
            result = (robot.turn_to_heading(90) if action == "turn" else
                      robot.drive_distance(500 if action == "forward" else -500, 0, 400))
            self.assertEqual(result.status, Result.STALLED)
            self.assertLess(hw.time, 2000)
            self.stopped(hw)

    def test_frozen_tracking_is_not_arrival(self):
        robot, hw = self.robot()
        hw.freeze_tracking = True
        result = robot.drive_distance(500, 0, 400)
        self.assertEqual(result.status, Result.STALLED)
        self.stopped(hw)

    def test_timeout_and_cancellation_stop_motors(self):
        robot, hw = self.robot()
        result = robot.drive_distance(2000, 0, 100, timeout_ms=100)
        self.assertEqual(result.status, Result.TIMED_OUT)
        self.stopped(hw)
        robot, hw = self.robot()
        hw.cancel_at = 200
        result = robot.turn_to_heading(90)
        self.assertEqual(result.status, Result.CANCELLED)
        self.stopped(hw)

    def test_sensor_disconnect(self):
        robot, hw = self.robot()
        hw.x_sensor.connected = False
        self.assertEqual(robot.drive_distance(100, 0, 100).status, Result.SENSOR_FAULT)
        self.stopped(hw)

    def test_tracking_jump_and_late_loop(self):
        robot, hw = self.robot()
        hw.y_sensor.deg = 100000
        hw.time += 20
        self.assertEqual(robot.drive_distance(100, 0, 100).status, Result.SENSOR_FAULT)
        robot, hw = self.robot()
        hw.step_ms = 300
        self.assertEqual(robot.turn_to_heading(90).status, Result.SENSOR_FAULT)

    def test_mechanism_completion_and_parallel_monitoring(self):
        robot, hw = self.robot()
        robot.lift.start(300, 80)
        robot.right_wing.start(200, 80)
        self.assertTrue(robot.drive_distance(400, 0, 300).ok())
        self.assertFalse(robot.lift.active)
        self.assertFalse(robot.right_wing.active)
        self.assertEqual(hw.lift[0].deg, 300)
        self.assertTrue(robot.wait_for(robot.lift).ok())

    def test_mechanism_timeout_and_skew(self):
        robot, hw = self.robot()
        hw.wings[1].frozen = True
        robot.right_wing.start(300, 80, timeout_ms=200)
        self.assertEqual(robot.wait_for(robot.right_wing).status, Result.TIMED_OUT)
        self.assertFalse(robot.right_wing.active)
        self.assertEqual(hw.wings[1].rpm, 0)
        robot, hw = self.robot()
        hw.lift[1].frozen = True
        robot.lift.start(300, 80)
        result = robot.drive_distance(500, 0, 300)
        self.assertEqual(result.status, Result.MECHANISM_FAULT)
        self.stopped(hw)
        self.assertFalse(robot.lift.active)

    def test_pid_reset_and_derivative_has_no_setpoint_kick(self):
        pid = code["PID"](1, kd=1, tau=0)
        self.assertEqual(pid.update(10, 0, .02, 100), 10)
        self.assertEqual(pid.update(20, 0, .03, 100), 20)
        self.assertEqual(pid.update(0, 1, .1, 100), -10)
        pid.reset()
        self.assertEqual(pid.update(0, 100, .02, 100), 0)

    def test_settling_requires_contiguous_low_error(self):
        settle = code["Settler"](100)
        self.assertFalse(settle.update(True, 0))
        self.assertFalse(settle.update(False, 80))
        self.assertFalse(settle.update(True, 100))
        self.assertFalse(settle.update(True, 180))
        self.assertTrue(settle.update(True, 200))

    def test_motor_rpm_conversion_and_common_saturation(self):
        class MixedSpec(Spec):
            DRIVE_MOTOR_TURNS_PER_WHEEL_TURN = (3, 1, 3, 3, 1, 3)
        robot, hw = self.robot(MixedSpec)
        robot.drive.command(2000, 1000)
        rpm = [motor.rpm for motor in hw.drive]
        self.assertAlmostEqual(rpm[0] / rpm[1], 3)
        self.assertAlmostEqual(rpm[0] / rpm[3], 2)
        self.assertTrue(all(abs(value) <= 3600 / ratio for value, ratio in
                            zip(rpm, MixedSpec.DRIVE_CARTRIDGE_RATIOS)))

    def test_route_aborts_before_release_on_motion_failure(self):
        robot, hw = self.robot()
        hw.blocked = True
        with self.assertRaises(Fault):
            try:
                code["run_route"](robot)
            finally:
                robot.stop_all()
        self.assertTrue(hw.claw.state)
        self.stopped(hw)
        self.assertTrue(all(not m.active for m in robot.mechanisms))

    def test_complete_route_and_repeated_absolute_mechanism_positions(self):
        robot, hw = self.robot()
        code["run_route"](robot)
        self.assertLess(hw.time, Config.ROUTE_TIMEOUT_MS)
        self.assertFalse(hw.claw.state)
        self.assertEqual(hw.lift[0].deg, 300)
        self.assertEqual(hw.wings[1].deg, 300)
        # Repeat after physically returning the base to its marked starting pose.
        hw.theta = hw.imu.deg = 0.0
        robot.begin_autonomous(lambda: hw.enabled)
        code["run_route"](robot)
        self.assertEqual(hw.lift[0].deg, 300)
        self.assertEqual(hw.wings[1].deg, 300)  # No extra 300-degree relative motion.

    def test_route_deadline_cancels_pending_mechanisms(self):
        class ShortConfig(Config):
            ROUTE_TIMEOUT_MS = 100
        robot, hw = self.robot(config=ShortConfig)
        robot.lift.start(300, 80)
        result = robot.wait_for(robot.lift)
        self.assertEqual(result.status, Result.TIMED_OUT)
        self.assertFalse(robot.lift.active)
        self.stopped(hw)

    def test_large_lateral_displacement_reports_path_error(self):
        robot, hw = self.robot()
        # Accumulate believable lateral movement over several samples.
        for _ in range(10):
            hw.x_sensor.deg += hw.tracking_degrees(20, "X")
            hw.time += 20
            robot.tick()
        # Next action starts a fresh line, so inject its displacement after it starts.
        original_pause = hw.pause
        def drift():
            original_pause()
            hw.x_sensor.deg += hw.tracking_degrees(20, "X")
        hw.pause = drift
        result = robot.drive_distance(500, 0, 300)
        self.assertEqual(result.status, Result.PATH_ERROR)
        self.stopped(hw)

    def test_heading_only_error_is_not_immediate_arrival(self):
        robot, hw = self.robot()
        hw.theta = math.radians(10)
        hw.imu.deg = 10
        robot.tracking.reset(0, 0, 10)
        result = robot.drive_distance(0, 0, 100)
        self.assertTrue(result.ok())
        self.assertGreater(hw.time, Config.SETTLE_MS)
        self.assertLess(abs(hw.imu.deg), 2)


if __name__ == "__main__":
    unittest.main()
