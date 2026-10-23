# Robot Parameters: Setup, Calibration, and Tuning

The parameters are divided into two groups:

1. `RobotSpec` describes what the robot physically is.
2. `ControlConfig` describes how aggressively and safely the software controls
   it.

Do not tune controller gains until the physical values in `RobotSpec` are
correct.

## Hardware values that must be set

| Parameter | What to set |
| --- | --- |
| `DRIVE_PORTS` | Six drive motor ports in this order: left front/mid/back, right front/mid/back |
| `DRIVE_REVERSED` | Whether each motor must reverse to make positive command move correctly |
| `DRIVE_CARTRIDGE_RATIOS` | Installed VEX cartridge ratios: `6`, `18`, or `36` |
| `LIFT_PORTS` | Lift motor ports |
| `LIFT_REVERSED` | Lift motor directions |
| `WING_PORTS` | Wing motor ports, left then right |
| `WING_REVERSED` | Wing motor directions |
| `X_SENSOR_PORT` | X tracking sensor port |
| `Y_SENSOR_PORT` | Y tracking sensor port |
| `IMU_PORT` | Inertial sensor port |
| `CLAW_THREE_WIRE_PORT` | Three-wire port used by the claw |
| `CLAW_CLOSED` | Electrical value that represents a closed claw |

The port and direction values are configuration, not controller tuning. Set
them from the actual robot wiring and verify each motor individually.

## Physical measurements that must be taken

### Tracking wheels

| Parameter | Measurement |
| --- | --- |
| `TRACK_X_DIAMETER_MM` | Effective rolling diameter of the X tracking wheel |
| `TRACK_Y_DIAMETER_MM` | Effective rolling diameter of the Y tracking wheel |
| `TRACK_X_SENSOR_TURNS_PER_WHEEL_TURN` | Sensor rotations per physical X wheel rotation |
| `TRACK_Y_SENSOR_TURNS_PER_WHEEL_TURN` | Sensor rotations per physical Y wheel rotation |
| `TRACK_X_ROTATION_MM_PER_RAD` | X-wheel travel during robot rotation divided by rotation in radians |
| `TRACK_Y_ROTATION_MM_PER_RAD` | Y-wheel travel during robot rotation divided by rotation in radians |

These values convert sensor rotations into millimeters and remove movement
caused purely by robot rotation.

### Drive wheels

| Parameter | Measurement |
| --- | --- |
| `DRIVE_WHEEL_DIAMETERS_MM` | Effective diameter of each drive wheel |
| `DRIVE_MOTOR_TURNS_PER_WHEEL_TURN` | Motor rotations required for one drive-wheel rotation |

The six entries must match this order:

```text
left front, left middle, left back,
right front, right middle, right back
```

Measure these under real load where possible. External gearing, wheel
compression, and traction can make nominal values inaccurate.

## Values to verify experimentally

| Parameter | Correct result when positive |
| --- | --- |
| `TRACK_X_SIGN` | Robot moves right and estimated X increases |
| `TRACK_Y_SIGN` | Robot moves forward and estimated Y increases |
| `IMU_SIGN` | Robot turns clockwise and estimated heading increases |

Use only `1` or `-1`. Verify these before testing autonomous paths.

## Field setup values

These values must match the robot's starting condition:

```python
LIFT_REFERENCE_DEG = (left_lift_reference, right_lift_reference)
WING_REFERENCE_DEG = (left_wing_reference, right_wing_reference)

START_X_MM = 0.0
START_Y_MM = 0.0
START_HEADING_DEG = 0.0
```

`LIFT_REFERENCE_DEG` and `WING_REFERENCE_DEG` are physical reference positions
used when the robot starts. They are not automatic homing.

The starting pose should be measured from the field setup:

- `START_X_MM`: starting position across the field
- `START_Y_MM`: starting position forward/backward
- `START_HEADING_DEG`: initial robot heading

For reliable autonomous runs, use a repeatable starting fixture or alignment
marks.

## Control tuning parameters

Tune these in `ControlConfig` after the physical calibration is complete.

### Basic loop and speed behavior

| Parameter | Purpose |
| --- | --- |
| `LOOP_MS` | Main control-loop period; normally leave at `20` ms |
| `MAX_SPEED_MM_S` | Maximum drive speed |
| `ACCEL_MM_S2` | How quickly commanded speed increases |
| `DECEL_MM_S2` | How quickly commanded speed decreases |
| `TURN_SLEW_MM_S2` | Limit on how quickly turning command changes |

Start with conservative speeds and acceleration. Increase them only after the
robot moves reliably.

### PID and steering gains

| Parameter | Purpose |
| --- | --- |
| `DISTANCE_KP`, `DISTANCE_KD` | Straight-line distance control |
| `HEADING_KP`, `HEADING_KD` | Heading correction while driving |
| `TURN_KP`, `TURN_KD` | In-place heading turns |
| `MAX_TURN_WHEEL_MM_S` | Maximum wheel speed during turns |
| `RATE_FILTER_TAU_S` | Filtering of measured speed and rotation rate |

General tuning symptoms:

- Slow response: increase proportional gain slightly.
- Oscillation: reduce proportional gain or increase damping.
- Overshoot: reduce gain or increase deceleration.
- Noisy response: increase filtering slightly, while avoiding excessive delay.

Tune one group at a time: straight driving, then turns, then path following.

### Accuracy and settling

| Parameter | Purpose |
| --- | --- |
| `DISTANCE_TOL_MM` | Acceptable distance error |
| `CROSS_TRACK_TOL_MM` | Acceptable lateral error |
| `HEADING_TOL_DEG` | Acceptable final heading error |
| `SETTLE_SPEED_MM_S` | Maximum speed considered stopped |
| `SETTLE_RATE_DEG_S` | Maximum rotation rate considered stopped |
| `SETTLE_MS` | Required stable duration |

Tightening these values improves accuracy but can cause timeouts or false
failures if the mechanical system is noisy.

### Stall and sensor safety

| Parameter | Purpose |
| --- | --- |
| `STALL_WINDOW_MS` | Time allowed without useful progress |
| `STALL_MIN_PROGRESS_MM` | Minimum translation considered progress |
| `STALL_MIN_ROTATION_DEG` | Minimum rotation considered progress |
| `STALL_COMMAND_MM_S` | Command level at which stall detection becomes active |
| `MAX_TRACK_SPEED_MM_S` | Maximum plausible tracking-wheel speed |
| `MAX_IMU_RATE_DEG_S` | Maximum plausible IMU rate |
| `SENSOR_JUMP_MARGIN_MM` | Allowed extra margin for sensor jumps |
| `MAX_LOOP_GAP_MS` | Maximum allowed control-loop delay |

Keep these conservative initially. Increase them only if normal operation
causes false sensor or stall faults.

### Timeouts and mechanism tuning

| Parameter | Purpose |
| --- | --- |
| `CALIBRATION_TIMEOUT_MS` | Maximum IMU calibration time |
| `ACTION_TIMEOUT_MS` | Default timeout for drive and turn actions |
| `ROUTE_TIMEOUT_MS` | Total autonomous route time |
| `MECHANISM_TIMEOUT_MS` | Lift and wing movement timeout |
| `MECHANISM_TOL_DEG` | Mechanism position tolerance |
| `MECHANISM_SETTLE_RATE_DEG_S` | Mechanism speed considered settled |
| `MECHANISM_SETTLE_MS` | Required mechanism settling duration |
| `LIFT_MAX_SKEW_DEG` | Maximum allowed difference between lift motors |
| `LIFT_SKEW_GRACE_MS` | Time allowed for temporary lift motor mismatch |

Set `ROUTE_TIMEOUT_MS` according to the competition time limit and route
requirements. Mechanism timeouts should be longer than normal movement time but
short enough to detect a disconnected or frozen motor.

## Navigation-specific tuning

These values are used by `follow_path()` and `go_to_pose()`.

| Parameter | Purpose |
| --- | --- |
| `NAV_TIMEOUT_MS` | Default navigation timeout |
| `NAV_LOOKAHEAD_MM` | Distance ahead on the path used as the steering target |
| `NAV_WAYPOINT_RADIUS_MM` | Acceptance radius for intermediate waypoints |
| `NAV_POSITION_TOL_MM` | Final position tolerance |
| `NAV_POSITION_KP` | Position-based speed response |
| `NAV_ALIGN_DEG` | Heading error above which the robot turns in place |
| `NAV_RECOVERY_CROSS_MM` | Cross-track distance that starts recovery |
| `NAV_MAX_CROSS_MM` | Maximum allowed cross-track error |
| `NAV_RECOVERY_SPEED_MM_S` | Maximum recovery speed |
| `NAV_RECOVERY_TIMEOUT_MS` | Maximum recovery duration |
| `NAV_RECOVERY_ATTEMPTS` | Maximum recovery attempts |
| `NAV_NO_PROGRESS_MS` | Time allowed without useful path progress |

## Recommended tuning order

1. Set the physical measurements.
2. Verify sensor signs and starting pose.
3. Tune straight-line driving.
4. Tune in-place turns.
5. Tune `NAV_LOOKAHEAD_MM` and `NAV_POSITION_KP`.
6. Tune recovery and stall thresholds.
7. Finally tighten tolerances and settling requirements.

The most important values to measure correctly are the tracking-wheel
diameters, tracking sensor ratios, tracking rotation offsets, drive-wheel
diameters, and drive gearing. Incorrect physical values cannot be fixed
reliably by changing PID gains.

## Measuring `TRACK_X_ROTATION_MM_PER_RAD`

`TRACK_X_ROTATION_MM_PER_RAD` compensates for X-tracking-wheel movement caused
by the robot rotating in place.

It is used in `Tracking.update()` approximately as:

```python
corrected_x_change = (
    measured_x_change
    - heading_change_rad * TRACK_X_ROTATION_MM_PER_RAD
)
```

The units are:

```text
millimeters of X-wheel travel per radian of robot rotation
```

Geometrically, it is approximately the signed distance between the X tracking
wheel and the robot's rotation center. It should be measured because wheel
alignment, mounting flex, and contact pressure affect the real value.

### Measurement procedure

First ensure these values are already correct:

```python
TRACK_X_DIAMETER_MM
TRACK_X_SENSOR_TURNS_PER_WHEEL_TURN
TRACK_X_SIGN
IMU_SIGN
```

Then:

1. Place the robot on its normal driving surface.
2. Keep the robot's center approximately fixed.
3. Reset the X tracking sensor and IMU readings.
4. Rotate the robot in place through several full turns.
5. Record the X tracking sensor movement and the IMU heading change.
6. Convert the X sensor movement into millimeters:

```text
x_travel_mm =
    sensor_degrees / 360
    × π
    × TRACK_X_DIAMETER_MM
    / TRACK_X_SENSOR_TURNS_PER_WHEEL_TURN
    × TRACK_X_SIGN
```

7. Convert heading change into radians:

```text
heading_change_rad =
    heading_change_degrees × π / 180 × IMU_SIGN
```

8. Calculate:

```text
TRACK_X_ROTATION_MM_PER_RAD =
    x_travel_mm / heading_change_rad
```

For example, if the X tracking wheel moves `120 mm` during a clockwise full
turn:

```text
TRACK_X_ROTATION_MM_PER_RAD = 120 / (2π) ≈ 19.1 mm/rad
```

Use several rotations and average the results. Measuring both clockwise and
counterclockwise rotations is better; the signed ratios should be similar.

With the current coordinate convention, the X tracking wheel is expected to be
ahead of the robot's center. During a positive clockwise rotation, it should
generally report positive X movement, so its value will usually be positive.

After setting the value, test an in-place rotation:

- Estimated X should return close to its starting value.
- If X drifts during rotation, adjust the value.
- If the correction makes the drift worse, reverse the value's sign or recheck
  `TRACK_X_SIGN` and `IMU_SIGN`.

This parameter is not the distance between the left and right drive wheels. It
describes the X tracking wheel's response to robot rotation.
