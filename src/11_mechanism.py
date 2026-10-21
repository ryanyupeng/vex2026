# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class Mechanism:
    """Absolute-position commands monitored even while the chassis moves."""
    def __init__(self, name, motors, hardware, config, max_skew=None):
        self.name, self.motors, self.hw, self.cfg = name, motors, hardware, config
        self.max_skew, self.active, self.target = max_skew, False, None

    def start(self, target_deg, speed_pct, timeout_ms=None):
        if self.active:
            raise ActionFault(Result.MECHANISM_FAULT, self.name + " already has an active move")
        self.target = target_deg
        self.started = self.hw.now()
        self.timeout = timeout_ms if timeout_ms is not None else self.cfg.MECHANISM_TIMEOUT_MS
        if (not finite(target_deg) or not finite(speed_pct) or not 0 < speed_pct <= 100
                or not finite(self.timeout) or self.timeout <= 0):
            raise ValueError("Invalid mechanism command")
        self.settler = Settler(self.cfg.MECHANISM_SETTLE_MS)
        self.skew_settler = Settler(self.cfg.LIFT_SKEW_GRACE_MS)
        self.active = True
        for motor in self.motors:
            motor.set_stopping(HOLD)
            motor.set_timeout(self.timeout, MSEC)
            motor.spin_to_position(target_deg, DEGREES, speed_pct, PERCENT, wait=False)

    def update(self):
        if not self.active:
            return
        now = self.hw.now()
        positions = [m.position(DEGREES) for m in self.motors]
        rates = [m.velocity(RPM) * 6.0 for m in self.motors]
        if not all(finite(v) for v in positions + rates):
            raise ActionFault(Result.SENSOR_FAULT, self.name + " invalid feedback")
        if self.max_skew is not None:
            skew = max(positions) - min(positions)
            if self.skew_settler.update(skew > self.max_skew, now):
                raise ActionFault(Result.MECHANISM_FAULT, self.name + " motors disagree")
        settled = (all(abs(p - self.target) <= self.cfg.MECHANISM_TOL_DEG for p in positions)
                   and all(abs(r) <= self.cfg.MECHANISM_SETTLE_RATE_DEG_S for r in rates))
        if self.settler.update(settled, now):
            self.stop()
        elif now - self.started >= self.timeout:
            raise ActionFault(Result.TIMED_OUT, self.name + " target not reached")

    def stop(self):
        for motor in self.motors:
            motor.stop(HOLD)
        self.active = False



# Development source fragment; build_vex.py concatenates fragments in filename order.
