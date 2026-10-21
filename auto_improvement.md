# Improvements Compared with the Original Python Code

## Structure and maintainability

- The monolithic program is divided into focused components for hardware,
  tracking, driving, mechanisms, navigation, results, and robot coordination.
- Robot ports, sensor signs, wheel geometry, and control parameters are grouped
  in `RobotSpec` and `ControlConfig`.
- Autonomous sample values are kept directly beside the route code that uses
  them, making field tuning easier.
- `build_vex.py` generates one deployable VEX Python file from the editable
  source fragments.

## Motion control

- Drive commands use calibrated wheel geometry and motor gearing to convert
  physical wheel speeds into motor RPM.
- Distance driving and heading turns use closed-loop control instead of simple
  timed motor commands.
- PID control, acceleration limits, settling checks, and stall detection improve
  repeatability and help detect blocked or ineffective movement.
- Tracking wheels and the IMU provide measured position, heading, speed, and
  angular-rate feedback.

## Navigation

- Added absolute field-coordinate navigation with `go_to_pose()` and
  `follow_path()`.
- Ordered waypoint paths support corners, duplicate points, final headings, and
  reverse travel.
- Navigation includes bounded path recovery, cross-track limits, timeout
  handling, and no-progress detection.
- Final position and heading are rechecked during settling instead of assuming
  that reaching the endpoint automatically means the action succeeded.

## Autonomous route reliability

- Route actions report explicit results and use `robot.require()` to stop the
  sequence when an action fails.
- Autonomous actions have timeouts and a shared route deadline.
- Mechanism movements are monitored while the robot drives, including lift motor
  skew, target position, settling, and timeout faults.
- Lift movements use absolute positions, reducing accumulated position drift.
- The claw and wing actions are coordinated with the route and are stopped on
  failures.

### Solve the reliability issue: Separate movement errors from localization errors 

| Problem                                                   | Planned response                                      |
| --------------------------------------------------------- | ----------------------------------------------------- |
| Friction or load changes actual speed                     | Wheel-speed feedback                                  |
| Robot drifts away from its intended heading               | Inertial heading feedback                             |
| Robot deviates from its path, with correct position estimate | Position/path feedback                            |
| Wheels slip and position estimate becomes wrong           | Independent position measurements or landmark corrections |
| Starting placement varies                                  | Repeatable setup and explicit pose initialization     |

The key reliability improvement is to avoid treating every route failure as
the same problem. A robot can fail to move as commanded, or it can move while
its estimate of position is wrong. These cases need different feedback and
different recovery decisions.

#### Movement errors

Movement errors describe the difference between the command sent to the drive
and the motion actually produced by the drivetrain. They can be caused by
friction, battery voltage, carpet, load, acceleration limits, a blocked wheel,
or unequal motor response.

The solution is to close the motion loop around measured response:

1. Convert the requested physical wheel speed into motor RPM using the measured
   wheel diameter, gearing, and motor configuration.
2. Use wheel-speed or motor-velocity feedback where available to detect whether
   the commanded speed is being produced.
3. Use tracking-wheel and IMU changes to verify that the robot is actually
   translating or rotating.
4. Detect insufficient progress over a bounded interval and return `STALLED`
   instead of allowing the route to continue indefinitely.
5. Stop the action when its position, heading, and velocity remain within the
   configured settling limits.

In the current implementation, `Drive.command()` performs the calibrated
physical-speed-to-RPM conversion, while the drive and navigation controllers
use progress watchdogs, tracking updates, IMU updates, acceleration limits, and
stall thresholds. This makes a blocked or ineffective movement an explicit
failure that the route can abort.

#### Localization errors

Localization errors describe a difference between the robot's estimated pose
and its true field pose. A robot may report that it reached a target while it
has actually drifted sideways, turned incorrectly, or accumulated error from
wheel slip.

The solution is to keep localization feedback separate from the drive command:

1. Estimate pose from independent X and Y tracking wheels and the IMU rather
   than assuming that a motor command equals a specific field displacement.
2. Compare the estimated pose with the active path segment, endpoint, and
   requested final heading.
3. Correct heading and cross-track error continuously during the action.
4. Use bounded recovery when the robot moves away from the active segment.
5. Recheck endpoint position and heading during the final settling interval.
6. Establish a repeatable starting pose and explicitly initialize the tracking
   origin and heading before autonomous movement.

The current navigation code therefore does not finish merely because a drive
command has run for a certain amount of time. It finishes only after measured
position, heading, translation rate, rotation rate, and settling duration all
meet the configured requirements.

There is an important limitation: if every tracking measurement slips in the
same way, internal sensors cannot discover the absolute error by themselves.
Independent measurements, field landmarks, or an external localization system
are required to correct that case. The software places limits on cross-track
error and reports a path failure, but it cannot infer an unknown shifted pose
without new information.

### Problems found in the original route and solutions

- **Timed driving did not prove that the robot reached its target.** The
  original route used timed or loosely bounded drive commands, including a
  500 ms forward command. The new route uses measured distance control with
  feedback, settling requirements, timeouts, and stall detection.
- **The original turn allowed a large heading error.** The route could continue
  after an incomplete turn because the requested 90-degree turn used a broad
  error allowance. The new `turn_to_heading()` action uses closed-loop heading
  control, configured heading tolerance, continuous settling, and explicit
  failure reporting.
- **Route failures did not reliably prevent later actions.** The original
  sequence did not consistently validate each movement before continuing. The
  new route checks every action with `robot.require()`, aborts immediately on a
  failure, and stops all motors in the failure path.
- **There was no shared autonomous time budget.** A route could spend too long
  in one action or continue after a deadline. The new implementation applies
  per-action timeouts and a whole-route deadline, including while mechanisms
  are being monitored.
- **Mechanism commands were not verified during the route.** The original code
  issued lift and wing commands without checking whether the target position was
  reached or whether paired lift motors disagreed. The new mechanism controller
  checks position, velocity, settling time, timeout, and lift skew while driving
  continues.
- **Legacy distance values had no explicit physical conversion.** The original
  route values represented encoder-scale units without a clear millimeter
  interpretation. The new route documents the legacy scale and converts it
  through the measured tracking-wheel diameter and sensor ratio, while allowing
  direct replacement with surveyed field distances later.
- **Absolute completion was not guaranteed after release actions.** The original
  route used relative lift commands, which could accumulate position error on
  repeated runs. The new route uses absolute lift targets for carry, release,
  and clearance positions.

## Hardware and startup safety

- Device presence, sensor validity, IMU calibration, and required measurements
  are checked before enabling the robot.
- Invalid parameters and nonfinite sensor readings are rejected.
- Motors are stopped during failures, cancellations, timeouts, mode changes,
  and normal action completion.
- Driver and autonomous modes share the same safety and motor-stop handling.

## Testing and documentation

- Added deterministic host-side tests for driving, turning, tracking,
  mechanisms, failures, navigation, recovery, and route deadlines.
- The generated deployment file is syntax-checked during the build.
- Documentation explains calibration requirements, coordinate conventions,
  navigation behavior, known limitations, and field-test procedures.
