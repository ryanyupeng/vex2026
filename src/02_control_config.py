# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class ControlConfig:
    # Starting tuning values; validate on the robot, not calibrated guarantees.
    LOOP_MS = 20
    MAX_LOOP_GAP_MS = 200
    CALIBRATION_TIMEOUT_MS = 8000
    MAX_SPEED_MM_S = 600.0
    ACCEL_MM_S2 = 600.0
    DECEL_MM_S2 = 800.0
    DISTANCE_KP = 3.0
    DISTANCE_KD = 0.15
    HEADING_KP = 8.0                 # wheel mm/s per degree
    HEADING_KD = 0.5                 # wheel mm/s per degree/s
    TURN_KP = 7.0
    TURN_KD = 0.6
    MAX_TURN_WHEEL_MM_S = 180.0
    TURN_SLEW_MM_S2 = 600.0
    RATE_FILTER_TAU_S = 0.06
    CROSS_TRACK_LOOKAHEAD_MM = 150.0
    MAX_STEERING_OFFSET_DEG = 15.0
    MAX_CROSS_TRACK_MM = 150.0
    DISTANCE_TOL_MM = 10.0
    CROSS_TRACK_TOL_MM = 15.0
    HEADING_TOL_DEG = 2.0
    SETTLE_SPEED_MM_S = 20.0
    SETTLE_RATE_DEG_S = 5.0
    SETTLE_MS = 150
    STALL_WINDOW_MS = 600
    STALL_MIN_PROGRESS_MM = 3.0
    STALL_MIN_ROTATION_DEG = 1.0
    STALL_COMMAND_MM_S = 35.0
    MAX_TRACK_SPEED_MM_S = 2500.0    # Plausibility limits, not commands.
    MAX_IMU_RATE_DEG_S = 800.0
    SENSOR_JUMP_MARGIN_MM = 3.0
    LOG_INTERVAL_MS = 200
    ACTION_TIMEOUT_MS = 5000
    ROUTE_TIMEOUT_MS = 15000         # Change for the intended competition period.
    MECHANISM_TIMEOUT_MS = 3500
    MECHANISM_TOL_DEG = 5.0
    MECHANISM_SETTLE_RATE_DEG_S = 15.0
    MECHANISM_SETTLE_MS = 100
    LIFT_MAX_SKEW_DEG = 15.0
    LIFT_SKEW_GRACE_MS = 150

    # Phase 5: field-coordinate polyline following with differential steering.
    NAV_TIMEOUT_MS = 10000
    NAV_LOOKAHEAD_MM = 120.0
    NAV_WAYPOINT_RADIUS_MM = 35.0
    NAV_POSITION_TOL_MM = 15.0
    NAV_POSITION_KP = 2.5
    NAV_ALIGN_DEG = 55.0         # Turn in place above this bearing error.
    NAV_RECOVERY_CROSS_MM = 100.0
    NAV_MAX_CROSS_MM = 300.0
    NAV_RECOVERY_SPEED_MM_S = 120.0
    NAV_RECOVERY_TIMEOUT_MS = 2000
    NAV_RECOVERY_ATTEMPTS = 2
    NAV_NO_PROGRESS_MS = 2500   # Also bounds small-command creep/oscillation.

