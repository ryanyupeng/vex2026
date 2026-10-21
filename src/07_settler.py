# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class Settler:
    def __init__(self, duration_ms):
        self.duration_ms, self.since = duration_ms, None

    def update(self, within_tolerance, now):
        if not within_tolerance:
            self.since = None
        elif self.since is None:
            self.since = now
        return self.since is not None and now - self.since >= self.duration_ms

# Development source fragment; build_vex.py concatenates fragments in filename order.
