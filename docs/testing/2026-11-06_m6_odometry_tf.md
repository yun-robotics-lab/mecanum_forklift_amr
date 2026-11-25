# 2026-11-06 - Milestone 6, odometry and TF

Setup: headless `mecanum_forklift_gazebo.launch.py gui:=false world:=empty.sdf`, commands via `/cmd_vel_teleop` (through `cmd_vel_gate`), RTF ~0.5. `/odom` (wheel_odometry, from `/joint_states`) compared with `/odom_ground_truth` (Gazebo model pose) as body-frame displacement from the same start. Scripts were run from a scratch dir.

| Check | Result | Status |
|---|---|---|
| `/tf` publishers | `robot_state_publisher`, `wheel_odometry` only; `/tf_static`: `robot_state_publisher` only | tested |
| `view_frames` | one connected tree `odom -> base_footprint -> base_link -> {4 wheels, fork_link -> front_camera_link -> front_camera_optical_frame, imu_link, lidar_link}`; one parent per frame | tested |
| Single motions (fwd/back/strafe L/R/diagonal at 0.2 and 0.5 m/s, 2 m or 2.83 m path) | position error 0.0 mm (<= 0.1 mm), yaw error 0.00 deg; criterion 2 % / 1 deg | tested, met |
| Rotation 360 deg at 0.5 rad/s ccw and 1.5 rad/s cw | yaw error 0.00 deg | tested, met |
| Circles (vx 0.2 + wz 0.4; strafe vy 0.2 + wz 0.4), one lap | position/yaw error 0.0 | tested, met |
| Closed 1 m square (no rotation, 0.2 and 0.5 m/s; with 90 deg turns) | odom vs GT error 0.0 mm. Return-to-start of the robot itself: 0.0 mm for the no-rotation squares, 11.5 mm / -0.41 deg for the rotating one (same in GT and odom; command/physics, not odometry) | tested, met (<= 2 cm, 2 deg) |
| Fork raised 0.12 m while driving 2 m (fwd, strafe) | odom error <= 0.1 mm | tested, met |
| Odom stamp vs `/clock` | -2..+4 ms (7772 msgs); criterion 50 ms | tested, met |
| TF extrapolation warnings | 0 in sim and EKF logs | tested |
| Static drift 60 s | 0.000 mm / 0.0000 deg (see 2026-11-06_m3-4_cleanup_step_b.md) | tested |
| EKF prototype (`robot_localization` ekf_node, scratch config: odom vx/vy/wz + IMU yaw rate, `publish_tf: false`) | same squares: 0.0 mm error, same as wheel odometry; 0 % improvement vs the 25 % criterion | tested, not adopted |

## Limits and caveats
- The zero errors are a property of the simulator: wheel contact has no slip (`mu2 = 0`, sphere collision), so joint increments equal body motion. This does not predict hardware (floor, roller slip, wheel wear). Hardware odometry drift is **unverified**.
- Because measured error is ~0, covariances could not be sized from data. `pose_variance`/`twist_variance` stay at the 0.01 placeholder (vy/yaw not enlarged); to be set from hardware measurements.
- The EKF test was a weak test for the same reason (the sim IMU has no noise configured, wheel odometry has no error to correct).
- Simulation does not validate physical safety.
