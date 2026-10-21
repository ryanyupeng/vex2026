# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class Drive:
    def __init__(self, hardware, spec, config):
        self.hw, self.spec, self.cfg = hardware, spec, config

    def command(self, left_mm_s, right_mm_s):
        speeds = (left_mm_s,) * 3 + (right_mm_s,) * 3
        rpm = [speed * 60.0 * ratio / (math.pi * diameter)
               for speed, ratio, diameter in zip(speeds,
                   self.spec.DRIVE_MOTOR_TURNS_PER_WHEEL_TURN,
                   self.spec.DRIVE_WHEEL_DIAMETERS_MM)]
        caps = [3600.0 / ratio for ratio in self.spec.DRIVE_CARTRIDGE_RATIOS]
        # Scale together so saturation preserves the requested steering ratio.
        scale = max([1.0, abs(left_mm_s) / self.cfg.MAX_SPEED_MM_S,
                     abs(right_mm_s) / self.cfg.MAX_SPEED_MM_S]
                    + [abs(value) / cap for value, cap in zip(rpm, caps)])
        for motor, value in zip(self.hw.drive, rpm):
            if abs(value) < 1e-6:
                motor.stop(BRAKE)
            else:
                motor.spin(FORWARD if value > 0 else REVERSE, abs(value) / scale, RPM)

    def stop(self):
        for motor in self.hw.drive:
            motor.stop(BRAKE)



# Development source fragment; build_vex.py concatenates fragments in filename order.
