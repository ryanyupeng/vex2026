# Manual Drive Logic Changes

The manual-drive behavior is not unchanged compared with
`hbase_original.v5python`.

## Behavior that remains equivalent

- Axis 3 controls forward/reverse movement.
- Axis 1 uses the same quadratic turn scaling:
  `axis1 * abs(axis1) / 100`.
- The same deadband threshold of `3` is used.
- Lift and wing button directions are preserved.

## Meaningful changes

- Drive output changed from direct percentage commands to calibrated mm/s to
  RPM conversion through `Drive.command()`.
- Joystick release now applies `BRAKE`; the original set velocity to zero and
  continued spinning forward.
- User control now runs only while driver control is active instead of using
  `while True`.
- Claw toggling moved from a background thread to edge detection in the driver
  loop. Press behavior is effectively similar, but the implementation differs.
- Readiness checks and `robot.stop_all()` were added.

The original implementation is in `hbase_original.v5python`, starting at
`user_control()`. The current implementation is in `src/14_main.py`.
