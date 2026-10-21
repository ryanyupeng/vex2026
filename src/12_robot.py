# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class Robot:
    def __init__(self, hardware, spec=RobotSpec, config=ControlConfig):
        self.hw, self.spec, self.cfg = hardware, spec, config
        self.drive = Drive(hardware, spec, config)
        self.tracking = Tracking(hardware, spec, config)
        self.navigator = Navigator(self)
        self.lift = Mechanism("lift", hardware.lift, hardware, config, config.LIFT_MAX_SKEW_DEG)
        self.left_wing = Mechanism("left wing", (hardware.wings[0],), hardware, config)
        self.right_wing = Mechanism("right wing", (hardware.wings[1],), hardware, config)
        self.mechanisms = (self.lift, self.left_wing, self.right_wing)
        self.ready = False
        self.active = lambda: False
        self.route_deadline = None
        self.last_log = -config.LOG_INTERVAL_MS

    def stop_all(self):
        self.drive.stop()
        for mechanism in self.mechanisms:
            mechanism.stop()

    def failure(self, status, detail):
        # Failure of any action cancels pending mechanism work as well as driving.
        self.stop_all()
        return Result(status, detail)

    def prepare(self):
        self.ready = False
        self.stop_all()
        errors = self.spec.validate()
        if errors:
            for error in errors:
                print("CONFIG: " + error)
            self.hw.show("Fill RobotSpec; autonomous not ready")
            return Result(Result.NOT_READY, "; ".join(errors))
        try:
            self.hw.check_devices()
            self.hw.show("Keep robot still: calibrating")
            self.hw.imu.calibrate()
            started = self.hw.now()
            while self.hw.imu.is_calibrating():
                if self.hw.now() - started >= self.cfg.CALIBRATION_TIMEOUT_MS:
                    raise ActionFault(Result.TIMED_OUT, "IMU calibration")
                self.hw.pause()
            for motor, position in zip(self.hw.lift, self.spec.LIFT_REFERENCE_DEG):
                motor.set_position(position, DEGREES)
            for motor, position in zip(self.hw.wings, self.spec.WING_REFERENCE_DEG):
                motor.set_position(position, DEGREES)
            self.hw.check_devices()
            self.ready = True
            self.hw.show("Ready: place at configured start pose")
            return Result(Result.ARRIVED)
        except ActionFault as fault:
            self.hw.show(fault.result.status + ": " + fault.result.detail)
            return fault.result

    def begin_autonomous(self, active):
        self.active = active
        self.stop_all()
        if not self.ready:
            raise ActionFault(Result.NOT_READY, "Calibration/configuration incomplete")
        if not active():
            raise ActionFault(Result.CANCELLED, "Autonomous is not enabled")
        self.hw.check_devices()
        self.tracking.reset(self.spec.START_X_MM, self.spec.START_Y_MM,
                            self.spec.START_HEADING_DEG)
        self.route_deadline = self.hw.now() + self.cfg.ROUTE_TIMEOUT_MS

    def tick(self):
        if not self.active():
            raise ActionFault(Result.CANCELLED, "Competition disabled or changed mode")
        if self.route_deadline is not None and self.hw.now() >= self.route_deadline:
            raise ActionFault(Result.TIMED_OUT, "Autonomous time budget exhausted")
        self.hw.check_devices()
        dt = self.tracking.update()
        for mechanism in self.mechanisms:
            mechanism.update()
        return dt

    def log(self, name, distance_error=0.0, cross_error=0.0):
        now = self.hw.now()
        if now - self.last_log >= self.cfg.LOG_INTERVAL_MS:
            self.last_log = now
            p = self.tracking
            print("%s t=%d x=%.1f y=%.1f h=%.1f dist_err=%.1f cross=%.1f speed=%.1f" %
                  (name, now, p.x, p.y, p.heading, distance_error, cross_error, p.speed))

    def drive_distance(self, distance_mm, heading_deg, max_speed_mm_s, timeout_ms=None):
        """Signed straight segment. X feedback steers out moderate lateral drift.

        Large lateral displacement near the endpoint may not be recoverable without
        another maneuver. Report failure instead of falsely declaring arrival.
        """
        timeout = self.cfg.ACTION_TIMEOUT_MS if timeout_ms is None else timeout_ms
        started = self.hw.now()
        p, c = self.tracking, self.cfg
        start_x, start_y = p.x, p.y
        heading = math.radians(heading_deg)
        distance_pid = PID(c.DISTANCE_KP, kd=c.DISTANCE_KD, tau=c.RATE_FILTER_TAU_S)
        heading_pid = PID(c.HEADING_KP, kd=c.HEADING_KD, tau=c.RATE_FILTER_TAU_S)
        settled = Settler(c.SETTLE_MS)
        speed = turn = 0.0
        stall_since, stall_progress = None, 0.0
        try:
            if (not finite(distance_mm) or not finite(heading_deg)
                    or not finite(max_speed_mm_s) or max_speed_mm_s <= 0
                    or not finite(timeout) or timeout <= 0):
                raise ValueError("Invalid drive parameters")
            limit = min(max_speed_mm_s, c.MAX_SPEED_MM_S)
            while True:
                dt = self.tick()
                now = self.hw.now()
                if now - started >= timeout:
                    return self.failure(Result.TIMED_OUT, "drive distance")
                if dt <= 0:
                    self.hw.pause()
                    continue
                dx, dy = p.x - start_x, p.y - start_y
                progress = dx * math.sin(heading) + dy * math.cos(heading)
                cross = dx * math.cos(heading) - dy * math.sin(heading)
                error = distance_mm - progress
                if abs(cross) > c.MAX_CROSS_TRACK_MM:
                    return self.failure(Result.PATH_ERROR, "Lateral displacement exceeds recovery limit")
                target_speed = distance_pid.update(error, progress, dt, limit)
                braking_limit = math.sqrt(2.0 * c.DECEL_MM_S2 * abs(error))
                target_speed = clamp(target_speed, -braking_limit, braking_limit)
                position_ok = abs(error) <= c.DISTANCE_TOL_MM and abs(cross) <= c.CROSS_TRACK_TOL_MM
                if position_ok:
                    target_speed = 0.0
                travel_sign = 1 if target_speed >= 0 else -1
                correction = clamp(math.degrees(math.atan2(-cross * travel_sign,
                                   c.CROSS_TRACK_LOOKAHEAD_MM)),
                                   -c.MAX_STEERING_OFFSET_DEG, c.MAX_STEERING_OFFSET_DEG)
                desired_heading = heading_deg if position_ok else heading_deg + correction
                target_turn = heading_pid.update(angle_error(desired_heading, p.heading),
                                                p.heading, dt, c.MAX_TURN_WHEEL_MM_S)
                if speed * target_speed < 0:  # Brake before reversing translation.
                    speed = approach(speed, 0.0, c.DECEL_MM_S2 * dt)
                else:
                    rate = c.ACCEL_MM_S2 if abs(target_speed) > abs(speed) else c.DECEL_MM_S2
                    speed = approach(speed, target_speed, rate * dt)
                turn = approach(turn, target_turn, c.TURN_SLEW_MM_S2 * dt)
                done = (position_ok and abs(angle_error(heading_deg, p.heading)) <= c.HEADING_TOL_DEG
                        and p.speed <= c.SETTLE_SPEED_MM_S
                        and abs(p.angular_rate) <= c.SETTLE_RATE_DEG_S)
                if settled.update(done, now):
                    return Result(Result.ARRIVED)
                if abs(speed) >= c.STALL_COMMAND_MM_S:
                    if stall_since is None or abs(progress - stall_progress) >= c.STALL_MIN_PROGRESS_MM:
                        stall_since, stall_progress = now, progress
                    elif now - stall_since >= c.STALL_WINDOW_MS:
                        return self.failure(Result.STALLED, "No tracking-wheel progress during drive")
                else:
                    stall_since = None
                self.drive.command(speed + turn, speed - turn)
                self.log("drive", error, cross)
                self.hw.pause()
        except ActionFault as fault:
            return self.failure(fault.result.status, fault.result.detail)
        finally:
            self.drive.stop()

    def turn_to_heading(self, target_deg, timeout_ms=None):
        c, p = self.cfg, self.tracking
        timeout = c.ACTION_TIMEOUT_MS if timeout_ms is None else timeout_ms
        started = self.hw.now()
        pid = PID(c.TURN_KP, kd=c.TURN_KD, tau=c.RATE_FILTER_TAU_S)
        settled = Settler(c.SETTLE_MS)
        command = 0.0
        stall_since, stall_heading = None, p.heading
        try:
            if not finite(target_deg) or not finite(timeout) or timeout <= 0:
                raise ValueError("Invalid turn parameters")
            while True:
                dt = self.tick()
                now = self.hw.now()
                if now - started >= timeout:
                    return self.failure(Result.TIMED_OUT, "turn")
                if dt <= 0:
                    self.hw.pause()
                    continue
                error = angle_error(target_deg, p.heading)
                desired = pid.update(error, p.heading, dt, c.MAX_TURN_WHEEL_MM_S)
                command = approach(command, desired, c.TURN_SLEW_MM_S2 * dt)
                if settled.update(abs(error) <= c.HEADING_TOL_DEG
                                  and abs(p.angular_rate) <= c.SETTLE_RATE_DEG_S
                                  and p.speed <= c.SETTLE_SPEED_MM_S, now):
                    return Result(Result.ARRIVED)
                if abs(command) >= c.STALL_COMMAND_MM_S:
                    if stall_since is None or abs(p.heading - stall_heading) >= c.STALL_MIN_ROTATION_DEG:
                        stall_since, stall_heading = now, p.heading
                    elif now - stall_since >= c.STALL_WINDOW_MS:
                        return self.failure(Result.STALLED, "No IMU rotation during turn")
                else:
                    stall_since = None
                self.drive.command(command, -command)
                self.log("turn")
                self.hw.pause()
        except ActionFault as fault:
            return self.failure(fault.result.status, fault.result.detail)
        finally:
            self.drive.stop()

    def wait_for(self, mechanism=None, duration_ms=0):
        """Keep tracking, cancellation and mechanism deadlines alive while waiting."""
        started = self.hw.now()
        try:
            while True:
                self.tick()
                if ((mechanism is None or not mechanism.active)
                        and self.hw.now() - started >= duration_ms):
                    return Result(Result.ARRIVED)
                self.hw.pause()
        except ActionFault as fault:
            return self.failure(fault.result.status, fault.result.detail)

    def require(self, result, step):
        print("%s: %s %s" % (step, result.status, result.detail))
        if not result.ok():
            raise ActionFault(result.status, step + ": " + result.detail)

    def set_claw(self, closed):
        self.tick()
        self.hw.claw.set(self.spec.CLAW_CLOSED if closed else not self.spec.CLAW_CLOSED)

    def go_to_pose(self, x_mm, y_mm, heading_deg=None, max_speed_mm_s=None,
                   reverse=False, timeout_ms=None):
        """Reach an absolute field position, then settle at the final heading."""
        return self.follow_path([(x_mm, y_mm)], heading_deg, max_speed_mm_s,
                                reverse, timeout_ms)

    def follow_path(self, waypoints, final_heading_deg=None, max_speed_mm_s=None,
                    reverse=False, timeout_ms=None):
        """Ordered absolute destinations; current pose is automatically the start."""
        return self.navigator.follow_path(waypoints, final_heading_deg,
                                          max_speed_mm_s, reverse, timeout_ms)



# Development source fragment; build_vex.py concatenates fragments in filename order.
