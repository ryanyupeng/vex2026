# Deployment source fragment; build_vex.py concatenates fragments in filename order.
def finite(value):
    return isinstance(value, (int, float)) and value == value and abs(value) < float("inf")


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def approach(current, target, amount):
    return current + clamp(target - current, -amount, amount)


