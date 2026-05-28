# C30D Integrated Architecture

This robot is wired around the C30D as the integrated low-level motor, steering,
encoder, and IMU controller.

## Actual Wiring

- Both drive motors connect directly to the C30D through 6-pin JST connectors.
- The steering servo connects directly to the C30D.
- The 12V traction battery connects to the C30D.
- The Raspberry Pi is powered separately from the C30D/motor power path.
- The IMU chip is integrated on the C30D, not wired as a separate Raspberry Pi I2C
  device.
- The C30D provides:
  - motor and steering-servo control,
  - encoder feedback from the drive motor connections,
  - onboard IMU feedback,
  - a feedback stream to the Raspberry Pi.

## Control Boundary

The Raspberry Pi is the robot supervisor and safety layer, but it is not directly wired
to the drive motors, steering servo, encoders, or C30D-integrated IMU. With the current
hardware wiring, the Pi can only command those low-level actuators and read those
integrated signals through the C30D interface.

## Why Bypassing The C30D Is Not Practical Now

Bypassing the C30D would require hardware changes, not just software changes:

- The drive motors and encoders terminate at the C30D 6-pin JST connectors.
- The steering servo terminates at the C30D.
- The C30D is the board connected to 12V motor power.
- The IMU is integrated on the C30D and is not exposed as a separate Pi-side IMU path in
  the current wiring.
- A Pi-side bypass would need separate motor drivers, encoder wiring, steering-servo
  wiring/power, and IMU wiring or a replacement IMU.

Therefore future motor and steering commands must go through the C30D unless the robot is
rewired.

## Current Software Status

- C30D feedback capture is read-only.
- C30D feedback frame structure is partially decoded.
- C30D feedback appears as fixed 24-byte frames with `0x7B` at byte 0 and `0x7D` at byte
  23.
- Candidate feedback fields exist for motion/IMU analysis, but field meanings remain
  provisional.
- The C30D command protocol is unknown.
- The real motor command path is disabled.
- Dry-run command tools produce only an `UNIMPLEMENTED` placeholder and do not open a
  C30D command path.

## Movement Blocking Rule

No real motor or steering movement should be attempted until the C30D command protocol is
discovered safely from official documentation, vendor examples, or known-good driver/demo
references. Command packet bytes must not be guessed from feedback frames.
