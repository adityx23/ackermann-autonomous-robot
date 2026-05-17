# Robot Safety Rules

This robot uses real motors, a steering servo, lidar, camera, IMU, and networked Jetson compute.

Rules:

1. No script should move motors on import.
2. Any motor test must require an explicit command-line flag such as --enable-motors.
3. All hardware movement scripts must support --dry-run.
4. Default speed and steering limits must be conservative.
5. If Jetson commands are stale, the Raspberry Pi must stop the robot.
6. If C30D communication fails, the Raspberry Pi must stop the robot.
7. The Raspberry Pi owns low-level safety. The Jetson is not trusted for emergency stopping.
8. Test motors with wheels lifted before driving on the floor.
