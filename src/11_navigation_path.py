"""Ordered field-coordinate line segments; geometry only, no VEX calls."""


class NavigationPath:
    def __init__(self, start, waypoints):
        self.points = [start]
        for point in waypoints:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("Each waypoint must be an (x_mm, y_mm) pair")
            if not all(finite(value) for value in point):
                raise ValueError("Waypoint coordinates must be finite")
            point = (float(point[0]), float(point[1]))
            # Consecutive duplicate points have no segment direction.
            if self.distance(point, self.points[-1]) > 1e-6:
                self.points.append(point)
        if len(self.points) == 1:
            self.points.append(start)  # Already at the target: may still need a turn.
        self.index = 0

    @staticmethod
    def distance(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def sample(self, position, lookahead, waypoint_radius):
        # Advance only through the current waypoint's acceptance circle. Never
        # search globally: an intersecting/returning path must retain its order.
        while self.index < len(self.points) - 2:
            if self.distance(position, self.points[self.index + 1]) > waypoint_radius:
                break
            self.index += 1
        a, b = self.points[self.index], self.points[self.index + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.sqrt(dx * dx + dy * dy)
        projection = 0.0 if length < 1e-6 else clamp(
            ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy) / length, 0.0, length)
        fraction = 0.0 if length < 1e-6 else projection / length
        closest = (a[0] + dx * fraction, a[1] + dy * fraction)
        fraction = 0.0 if length < 1e-6 else min(length, projection + lookahead) / length
        target = (a[0] + dx * fraction, a[1] + dy * fraction)
        cross = self.distance(position, closest)
        remaining = self.distance(position, b)
        for i in range(self.index + 1, len(self.points) - 1):
            remaining += self.distance(self.points[i], self.points[i + 1])
        return target, closest, cross, remaining

    def at_last_segment(self):
        return self.index == len(self.points) - 2
