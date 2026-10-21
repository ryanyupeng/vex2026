# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class Hardware:
    def __init__(self, spec):
        self.brain = Brain()
        self.controller = Controller(PRIMARY)
        gears = {6: GearSetting.RATIO_6_1, 18: GearSetting.RATIO_18_1,
                 36: GearSetting.RATIO_36_1}
        self.drive = tuple(Motor(getattr(Ports, "PORT" + str(port)), gears[ratio], reverse)
                           for port, ratio, reverse in zip(spec.DRIVE_PORTS,
                               spec.DRIVE_CARTRIDGE_RATIOS, spec.DRIVE_REVERSED))
        self.lift = tuple(Motor(getattr(Ports, "PORT" + str(port)), gears[36], reverse)
                          for port, reverse in zip(spec.LIFT_PORTS, spec.LIFT_REVERSED))
        self.wings = tuple(Motor(getattr(Ports, "PORT" + str(port)), gears[18], reverse)
                           for port, reverse in zip(spec.WING_PORTS, spec.WING_REVERSED))
        self.x_sensor = Rotation(getattr(Ports, "PORT" + str(spec.X_SENSOR_PORT)), False)
        self.y_sensor = Rotation(getattr(Ports, "PORT" + str(spec.Y_SENSOR_PORT)), False)
        self.imu = Inertial(getattr(Ports, "PORT" + str(spec.IMU_PORT)))
        self.claw = DigitalOut(getattr(self.brain.three_wire_port, spec.CLAW_THREE_WIRE_PORT))
        self.clock = Timer()          # Never clear this shared elapsed-time source.

    def now(self):
        return self.clock.time(MSEC)

    def pause(self):
        wait(ControlConfig.LOOP_MS, MSEC)

    def check_devices(self):
        devices = self.drive + self.lift + self.wings + (self.x_sensor, self.y_sensor, self.imu)
        # Some VEX Python releases do not expose installed() on every device.
        # Use it when available; plausibility/progress checks still run every tick.
        for device in devices:
            checker = getattr(device, "installed", None)
            if checker is not None and not checker():
                raise ActionFault(Result.SENSOR_FAULT, "A configured motor/sensor is disconnected")
        if self.imu.is_calibrating():
            raise ActionFault(Result.SENSOR_FAULT, "IMU calibration is still active")

    def show(self, message):
        print(message)
        self.brain.screen.clear_screen()
        self.brain.screen.print(message)



# Development source fragment; build_vex.py concatenates fragments in filename order.
