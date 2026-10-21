# Deployment source fragment; build_vex.py concatenates fragments in filename order.
def run_route(robot):
    """Sample task sequence, now with precise turns and checked outcomes.

    The old 500 ms drive is distance travel here, NOT an inferred contact/push.
    Survey the distances and validate the now fully completed 90-degree turn.
    """
    # Tune the sample route here as the field route changes. These values are
    # deliberately local to the route instead of hidden behind a config class.
    legacy_sensor_turns_per_unit = 0.31
    first_leg_units = 4.0
    second_leg_units = 5.0
    final_approach_units = 5.0
    retreat_units = 2.0
    last_leg_units = 3.0
    first_turn_deg = 90.0
    last_turn_deg = 50.0
    lift_carry_deg = 300.0
    lift_release_deg = 100.0
    lift_clear_deg = 150.0
    right_wing_deploy_deg = 300.0
    mechanism_speed_pct = 80

    c, s = robot.cfg, robot.spec
    unit_mm = (legacy_sensor_turns_per_unit * math.pi * s.TRACK_Y_DIAMETER_MM
               / s.TRACK_Y_SENSOR_TURNS_PER_WHEEL_TURN)
    robot.set_claw(True)
    robot.lift.start(lift_carry_deg, mechanism_speed_pct)
    robot.right_wing.start(right_wing_deploy_deg, mechanism_speed_pct)
    robot.require(robot.wait_for(robot.right_wing), "deploy right wing")
    robot.require(robot.drive_distance(first_leg_units * unit_mm,
                  s.START_HEADING_DEG, 0.7 * c.MAX_SPEED_MM_S), "first leg")
    robot.require(robot.turn_to_heading(first_turn_deg), "first turn")
    robot.require(robot.drive_distance(second_leg_units * unit_mm,
                  first_turn_deg, c.MAX_SPEED_MM_S), "second leg")
    robot.require(robot.drive_distance(final_approach_units * unit_mm,
                  first_turn_deg, 0.5 * c.MAX_SPEED_MM_S), "final approach")
    robot.require(robot.wait_for(robot.lift), "finish carry lift")
    robot.lift.start(lift_release_deg, mechanism_speed_pct)
    robot.require(robot.wait_for(robot.lift), "lower before release")
    robot.set_claw(False)
    robot.require(robot.wait_for(duration_ms=150), "claw release dwell")
    robot.lift.start(lift_clear_deg, mechanism_speed_pct)
    robot.require(robot.wait_for(robot.lift), "clear released object")
    robot.lift.start(lift_carry_deg, mechanism_speed_pct)
    robot.require(robot.drive_distance(-retreat_units * unit_mm,
                  first_turn_deg, 0.7 * c.MAX_SPEED_MM_S), "retreat")
    robot.require(robot.turn_to_heading(last_turn_deg), "last turn")
    robot.require(robot.drive_distance(last_leg_units * unit_mm,
                  last_turn_deg, 0.3 * c.MAX_SPEED_MM_S), "last leg")
    robot.require(robot.wait_for(robot.lift), "finish lift")

# Development source fragment; build_vex.py concatenates fragments in filename order.
