# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class Tracking:
    """Both passive tracking wheels measure pose; no sideways drive is assumed."""
    def __init__(self, hardware, spec, config):
        self.hw, self.spec, self.cfg = hardware, spec, config
        self.x = self.y = self.heading = self.speed = self.angular_rate = 0.0
        self.last_time = None

    def readings(self):
        s = self.spec
        x = self.hw.x_sensor.position(DEGREES) / 360.0 * math.pi * s.TRACK_X_DIAMETER_MM
        y = self.hw.y_sensor.position(DEGREES) / 360.0 * math.pi * s.TRACK_Y_DIAMETER_MM
        x *= s.TRACK_X_SIGN / s.TRACK_X_SENSOR_TURNS_PER_WHEEL_TURN
        y *= s.TRACK_Y_SIGN / s.TRACK_Y_SENSOR_TURNS_PER_WHEEL_TURN
        heading = self.hw.imu.rotation(DEGREES) * s.IMU_SIGN
        if not all(finite(v) for v in (x, y, heading)):
            raise ActionFault(Result.SENSOR_FAULT, "Nonfinite tracking/IMU measurement")
        return x, y, heading

    def reset(self, x, y, heading):
        self.last_x, self.last_y, raw_heading = self.readings()
        self.heading_offset = heading - raw_heading
        self.x, self.y, self.heading = x, y, heading
        self.last_time = self.hw.now()
        self.speed = self.angular_rate = 0.0

    def update(self):
        now = self.hw.now()
        elapsed = now - self.last_time
        if elapsed <= 0:
            return 0.0
        if elapsed > self.cfg.MAX_LOOP_GAP_MS:
            raise ActionFault(Result.SENSOR_FAULT, "Control loop missed its timing budget")
        dt = elapsed / 1000.0
        x, y, raw_heading = self.readings()
        heading = raw_heading + self.heading_offset
        delta_deg = heading - self.heading
        delta = math.radians(delta_deg)
        dx = x - self.last_x - delta * self.spec.TRACK_X_ROTATION_MM_PER_RAD
        dy = y - self.last_y - delta * self.spec.TRACK_Y_ROTATION_MM_PER_RAD
        travel = math.sqrt(dx * dx + dy * dy)
        if (travel > self.cfg.MAX_TRACK_SPEED_MM_S * dt + self.cfg.SENSOR_JUMP_MARGIN_MM
                or abs(delta_deg) > self.cfg.MAX_IMU_RATE_DEG_S * dt):
            raise ActionFault(Result.SENSOR_FAULT, "Implausible tracking/IMU jump")
        arc = 1.0 if abs(delta) < 1e-6 else 2.0 * math.sin(delta / 2.0) / delta
        mid = math.radians(self.heading) + delta / 2.0
        self.x += arc * (dx * math.cos(mid) + dy * math.sin(mid))
        self.y += arc * (-dx * math.sin(mid) + dy * math.cos(mid))
        alpha = dt / (self.cfg.RATE_FILTER_TAU_S + dt)
        self.speed += alpha * (travel / dt - self.speed)
        self.angular_rate += alpha * (delta_deg / dt - self.angular_rate)
        self.heading = heading
        self.last_x, self.last_y, self.last_time = x, y, now
        return dt

# Development source fragment; build_vex.py concatenates fragments in filename order.
