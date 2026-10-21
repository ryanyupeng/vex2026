# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class PID:
    """Measured seconds, filtered derivative-on-measurement, conditional integration."""
    def __init__(self, kp, ki=0.0, kd=0.0, integral_limit=0.0, tau=0.06):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit, self.tau = integral_limit, tau
        self.reset()

    def reset(self):
        self.integral = self.rate = 0.0
        self.previous = None

    def update(self, error, measurement, dt, limit):
        if dt <= 0:
            raise ValueError("PID dt must be positive")
        raw_rate = 0.0 if self.previous is None else (measurement - self.previous) / dt
        self.previous = measurement
        self.rate += dt / (self.tau + dt) * (raw_rate - self.rate)
        candidate = clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        output = self.kp * error + self.ki * candidate - self.kd * self.rate
        if abs(output) <= limit or error * output < 0:
            self.integral = candidate
        return clamp(self.kp * error + self.ki * self.integral - self.kd * self.rate,
                     -limit, limit)


