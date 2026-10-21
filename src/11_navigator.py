"""Dead-reckoning navigation using the existing shared motion coordinator."""


class Navigator:
    def __init__(self, robot):
        self.robot = robot

    def follow_path(self, waypoints, final_heading_deg=None,
                    max_speed_mm_s=None, reverse=False, timeout_ms=None):
        """Follow ordered absolute (x,y) destinations. Start is prepended automatically.

        Uses a moving lookahead point and heading PD, not the X-drive mixer.
        Corner waypoints are accepted within NAV_WAYPOINT_RADIUS_MM; use separate
        go_to_pose calls for exact stops. No obstacle sensing or global localization.
        """
        r, c, p = self.robot, self.robot.cfg, self.robot.tracking
        speed_limit = c.MAX_SPEED_MM_S if max_speed_mm_s is None else max_speed_mm_s
        timeout = c.NAV_TIMEOUT_MS if timeout_ms is None else timeout_ms
        try:
            if not r.ready or p.last_time is None:
                return r.failure(Result.NOT_READY, "Begin autonomous before navigation")
            if (not isinstance(waypoints, (list, tuple)) or not waypoints
                    or not finite(speed_limit) or speed_limit <= 0
                    or not finite(timeout) or timeout <= 0
                    or not isinstance(reverse, bool)
                    or (final_heading_deg is not None and not finite(final_heading_deg))):
                raise ValueError("Invalid path, heading, speed, reverse flag or timeout")
            path = NavigationPath((p.x, p.y), waypoints)
            speed_limit = min(speed_limit, c.MAX_SPEED_MM_S)
            direction = -1.0 if reverse else 1.0
            pid = PID(c.HEADING_KP, kd=c.HEADING_KD, tau=c.RATE_FILTER_TAU_S)
            settled = Settler(c.SETTLE_MS)
            started = r.hw.now()
            speed = turn = 0.0
            recovery_since, recovery_count = None, 0
            previous_index = -1
            best_remaining = best_angle = float("inf")
            progress_since = started
            previous_mode = None
            stall_since = None
            stall_position = (p.x, p.y)
            stall_heading = p.heading
            end_heading = final_heading_deg

            while True:
                dt = r.tick()  # Services X/Y tracking, all mechanisms, cancellation.
                now = r.hw.now()
                if now - started >= timeout:
                    return r.failure(Result.TIMED_OUT, "Navigation action deadline")
                if dt <= 0:
                    r.hw.pause()
                    continue
                position = (p.x, p.y)
                target, closest, cross, remaining = path.sample(
                    position, c.NAV_LOOKAHEAD_MM, c.NAV_WAYPOINT_RADIUS_MM)
                distance = path.distance(position, path.points[-1])
                if cross > c.NAV_MAX_CROSS_MM:
                    return r.failure(Result.PATH_ERROR, "Too far from active path segment")

                if cross > c.NAV_RECOVERY_CROSS_MM and recovery_since is None:
                    recovery_since = now
                    recovery_count += 1
                    if recovery_count > c.NAV_RECOVERY_ATTEMPTS:
                        return r.failure(Result.PATH_ERROR, "Path recovery attempts exhausted")
                if recovery_since is not None:
                    if cross <= c.NAV_RECOVERY_CROSS_MM * 0.5:
                        recovery_since = None
                    elif now - recovery_since >= c.NAV_RECOVERY_TIMEOUT_MS:
                        return r.failure(Result.PATH_ERROR, "Path recovery timed out")
                    else:
                        target = closest  # Rejoin at low speed; do not skip ahead.

                position_ok = path.at_last_segment() and distance <= c.NAV_POSITION_TOL_MM
                if position_ok:
                    # First stop translation, then finish the requested heading.
                    if end_heading is None:
                        end_heading = p.heading
                    desired_heading = end_heading
                    desired_speed = 0.0
                    mode = "finish"
                else:
                    if final_heading_deg is None:
                        end_heading = None
                    dx, dy = target[0] - p.x, target[1] - p.y
                    desired_heading = math.degrees(math.atan2(dx, dy)) + (180.0 if reverse else 0.0)
                    bearing_error = angle_error(desired_heading, p.heading)
                    # Slow for corners and endpoints; don't drive away while facing
                    # the wrong direction. Rotation can reorient after an overshoot.
                    available = min(speed_limit, c.NAV_POSITION_KP * remaining,
                                    math.sqrt(2.0 * c.DECEL_MM_S2 * remaining))
                    if recovery_since is not None:
                        available = min(available, c.NAV_RECOVERY_SPEED_MM_S)
                    if abs(bearing_error) >= c.NAV_ALIGN_DEG:
                        desired_speed = 0.0
                        mode = "align"
                    else:
                        desired_speed = direction * available * max(0.0, math.cos(math.radians(bearing_error)))
                        mode = "travel"

                error = angle_error(desired_heading, p.heading)
                desired_turn = pid.update(error, p.heading, dt, c.MAX_TURN_WHEEL_MM_S)
                if position_ok and p.speed > c.SETTLE_SPEED_MM_S:
                    desired_turn = 0.0
                rate = c.ACCEL_MM_S2 if abs(desired_speed) > abs(speed) else c.DECEL_MM_S2
                speed = approach(speed, desired_speed, rate * dt)
                turn = approach(turn, desired_turn, c.TURN_SLEW_MM_S2 * dt)
                done = (position_ok and abs(error) <= c.HEADING_TOL_DEG
                        and p.speed <= c.SETTLE_SPEED_MM_S
                        and abs(p.angular_rate) <= c.SETTLE_RATE_DEG_S)
                if settled.update(done, now):
                    r.log("nav arrived", distance, cross)
                    return Result(Result.ARRIVED)

                # A monotonic best-distance/angle watchdog catches creep and loops.
                # Reset only at a waypoint or a change between travel and turning.
                if path.index != previous_index or mode != previous_mode:
                    progress_since = now
                    best_remaining, best_angle = remaining, abs(error)
                    previous_index, previous_mode = path.index, mode
                improved = ((mode == "travel" and remaining < best_remaining - c.STALL_MIN_PROGRESS_MM)
                            or (mode != "travel" and abs(error) < best_angle - c.STALL_MIN_ROTATION_DEG))
                if improved:
                    progress_since = now
                    best_remaining, best_angle = min(best_remaining, remaining), min(best_angle, abs(error))
                if not done and now - progress_since >= c.NAV_NO_PROGRESS_MS:
                    return r.failure(Result.STALLED, "No useful navigation progress")

                # Faster detection of a commanded move/turn that produces no motion.
                translating = abs(speed) >= c.STALL_COMMAND_MM_S
                rotating = abs(turn) >= c.STALL_COMMAND_MM_S
                moved = path.distance(position, stall_position) >= c.STALL_MIN_PROGRESS_MM
                turned = abs(p.heading - stall_heading) >= c.STALL_MIN_ROTATION_DEG
                if translating or rotating:
                    if stall_since is None or (moved if translating else turned):
                        stall_since, stall_position, stall_heading = now, position, p.heading
                    elif now - stall_since >= c.STALL_WINDOW_MS:
                        return r.failure(Result.STALLED, "No motion during navigation")
                else:
                    stall_since = None
                r.drive.command(speed + turn, speed - turn)
                label = "rejoin%d %s" % (recovery_count, mode) if recovery_since is not None else mode
                r.log("nav %s segment=%d" % (label, path.index), distance, cross)
                r.hw.pause()
        except ActionFault as fault:
            return r.failure(fault.result.status, fault.result.detail)
        except Exception:
            r.stop_all()
            raise
        finally:
            r.drive.stop()
