"""Navigation sample; points are absolute field coordinates."""


def run_navigation_example(robot):
    # Tune the navigation sample here as the test field changes.
    points_mm = ((0.0, 400.0), (300.0, 600.0))
    final_heading_deg = 90.0
    speed_mm_s = 250.0
    timeout_ms = 10000

    robot.require(robot.follow_path(
        points_mm,
        final_heading_deg=final_heading_deg,
        max_speed_mm_s=speed_mm_s,
        timeout_ms=timeout_ms,
    ), "navigation example")
