# Robot hardware specification.
"""Self-contained VEXcode program. Fill RobotSpec using README.md before running.

Coordinates: +X right, +Y forward at zero heading; clockwise angles positive.
All navigation uses mm, degrees and measured seconds. No external sensor needed.
"""
from vex import *
import math


class RobotSpec:
    # Unknown dimensions deliberately have no guessed default. Zero is a valid offset.
    TRACK_X_DIAMETER_MM = None
    TRACK_Y_DIAMETER_MM = None
    TRACK_X_SENSOR_TURNS_PER_WHEEL_TURN = 1.0
    TRACK_Y_SENSOR_TURNS_PER_WHEEL_TURN = 1.0
    TRACK_X_SIGN = 1                 # Sensor travel to the right is positive.
    TRACK_Y_SIGN = 1                 # Forward travel is positive.
    IMU_SIGN = 1                     # Clockwise rotation is positive.
    # Signed tracking travel / clockwise rotation in radians during a pure turn.
    # X: positive ahead of center. Y: negative to the right of center.
    TRACK_X_ROTATION_MM_PER_RAD = None
    TRACK_Y_ROTATION_MM_PER_RAD = None

    # Order: left front/mid/back, right front/mid/back. External gearing ONLY.
    DRIVE_WHEEL_DIAMETERS_MM = (None, None, None, None, None, None)
    DRIVE_MOTOR_TURNS_PER_WHEEL_TURN = (None, None, None, None, None, None)
    DRIVE_PORTS = (2, 1, 14, 12, 10, 13)
    DRIVE_REVERSED = (True, False, True, False, True, False)
    DRIVE_CARTRIDGE_RATIOS = (6, 18, 6, 6, 18, 6)
    LIFT_PORTS = (11, 19)
    LIFT_REVERSED = (False, True)
    WING_PORTS = (15, 16)             # Left, right.
    WING_REVERSED = (True, False)
    X_SENSOR_PORT = 3
    Y_SENSOR_PORT = 9
    IMU_PORT = 17
    CLAW_THREE_WIRE_PORT = "h"
    CLAW_CLOSED = True

    # Manually place mechanisms at these references before program startup.
    # Encoder assignment is NOT physical homing.
    LIFT_REFERENCE_DEG = (0.0, 0.0)
    WING_REFERENCE_DEG = (0.0, 0.0)
    START_X_MM = 0.0
    START_Y_MM = 0.0
    START_HEADING_DEG = 0.0

    @classmethod
    def validate(cls):
        errors = []
        for name in ("TRACK_X_DIAMETER_MM", "TRACK_Y_DIAMETER_MM",
                     "TRACK_X_SENSOR_TURNS_PER_WHEEL_TURN",
                     "TRACK_Y_SENSOR_TURNS_PER_WHEEL_TURN"):
            value = getattr(cls, name)
            if not finite(value) or value <= 0:
                errors.append(name + " must be measured and positive")
        for name in ("TRACK_X_ROTATION_MM_PER_RAD", "TRACK_Y_ROTATION_MM_PER_RAD"):
            if not finite(getattr(cls, name)):
                errors.append(name + " must be measured (zero is allowed)")
        for name in ("TRACK_X_SIGN", "TRACK_Y_SIGN", "IMU_SIGN"):
            if getattr(cls, name) not in (-1, 1):
                errors.append(name + " must be +1 or -1")
        for name in ("DRIVE_WHEEL_DIAMETERS_MM", "DRIVE_MOTOR_TURNS_PER_WHEEL_TURN"):
            values = getattr(cls, name)
            if len(values) != 6 or any(not finite(v) or v <= 0 for v in values):
                errors.append(name + " needs six positive measurements")
        for name in ("DRIVE_PORTS", "DRIVE_REVERSED", "DRIVE_CARTRIDGE_RATIOS"):
            if len(getattr(cls, name)) != 6:
                errors.append(name + " needs six entries")
        if any(r not in (6, 18, 36) for r in cls.DRIVE_CARTRIDGE_RATIOS):
            errors.append("Unsupported drive cartridge ratio")
        if any(len(getattr(cls, name)) != 2 for name in
               ("LIFT_PORTS", "LIFT_REVERSED", "WING_PORTS", "WING_REVERSED",
                "LIFT_REFERENCE_DEG", "WING_REFERENCE_DEG")):
            errors.append("Lift/wing configuration needs two entries per setting")
        return errors


