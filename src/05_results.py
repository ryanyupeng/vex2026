# Deployment source fragment; build_vex.py concatenates fragments in filename order.
class Result:
    ARRIVED = "ARRIVED"
    TIMED_OUT = "TIMED_OUT"
    STALLED = "STALLED"
    CANCELLED = "CANCELLED"
    SENSOR_FAULT = "SENSOR_FAULT"
    PATH_ERROR = "PATH_ERROR"
    NOT_READY = "NOT_READY"
    MECHANISM_FAULT = "MECHANISM_FAULT"

    def __init__(self, status, detail=""):
        self.status, self.detail = status, detail

    def ok(self):
        return self.status == self.ARRIVED


class ActionFault(Exception):
    def __init__(self, status, detail):
        self.result = Result(status, detail)


