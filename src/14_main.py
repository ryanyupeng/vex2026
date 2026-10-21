# Deployment source fragment; build_vex.py concatenates fragments in filename order.
AUTONOMOUS_ROUTE = "sample"  # "sample" or "navigation_example"


def main():
    hw = Hardware(RobotSpec)
    robot = Robot(hw)
    competition = None

    def autonomous():
        try:
            robot.begin_autonomous(lambda: competition.is_enabled() and competition.is_autonomous())
            if AUTONOMOUS_ROUTE == "sample":
                run_route(robot)
            elif AUTONOMOUS_ROUTE == "navigation_example":
                run_navigation_example(robot)
            else:
                raise ValueError("Unknown AUTONOMOUS_ROUTE")
            hw.show("Autonomous ARRIVED")
        except ActionFault as fault:
            hw.show(fault.result.status + ": " + fault.result.detail)
        except Exception as error:
            hw.show("Autonomous error: " + str(error))
        finally:
            robot.stop_all()

    def user_control():
        # No persistent claw thread: only the active mode owns actuator commands.
        if not robot.ready:
            hw.show("NOT_READY: check RobotSpec/calibration")
            return
        robot.stop_all()
        previous_claw_button = False
        try:
            while competition.is_enabled() and competition.is_driver_control():
                a3 = hw.controller.axis3.position()
                a1 = hw.controller.axis1.position()
                turn = a1 * abs(a1) / 100.0
                if abs(a3) + abs(turn) <= 3:
                    robot.drive.stop()
                else:
                    scale = robot.cfg.MAX_SPEED_MM_S / 100.0
                    robot.drive.command((a3 + turn) * scale, (a3 - turn) * scale)
                pressed = hw.controller.buttonR1.pressing()
                if pressed and not previous_claw_button:
                    hw.claw.set(not hw.claw.value())
                previous_claw_button = pressed
                for motor in hw.lift:
                    if hw.controller.buttonL1.pressing():
                        motor.spin(FORWARD, 100, PERCENT)
                    elif hw.controller.buttonL2.pressing():
                        motor.spin(REVERSE, 100, PERCENT)
                    else:
                        motor.stop(HOLD)
                for motor in hw.wings:
                    if hw.controller.buttonR2.pressing():
                        motor.spin(FORWARD, 100, PERCENT)
                    else:
                        motor.stop(HOLD)
                hw.pause()
        finally:
            robot.stop_all()

    # Calibration completes before competition callbacks are installed.
    wait(200, MSEC)  # Allow smart-port devices to initialize before presence checks.
    robot.prepare()
    competition = Competition(user_control, autonomous)
    while True:
        wait(100, MSEC)


if __name__ == "__main__":
    main()
