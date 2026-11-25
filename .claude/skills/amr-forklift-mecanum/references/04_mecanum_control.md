# 04 - Mecanum Control

## 1. Purpose
Kinematics, wheel conventions, controller options and tests for the 4-wheel mecanum base. **This robot is holonomic; never reuse differential-drive code, limits or Nav2 motion models unchanged.**

## 2. Geometry and conventions (current model, from Xacro)
- Frame: x forward, y left, z up (REP-103). Wheels: FL (+x,+y), FR (+x,-y), RL (-x,+y), RR (-x,-y). lx = 0.11 m, ly = 0.1305 m, r = 0.039 m. All wheel joints use axis `0 1 0`.
- **Derived (math, verify by test)**: rotation about +Y by right-hand rule moves the wheel top toward +x, so **positive joint velocity = wheel driving the robot forward** for all four wheels. If so, no per-wheel sign flip is needed in the controller.
- Roller pattern: the Gazebo friction setup assumes the "X" pattern (viewed from above): `fdir1` FL `1 -1 0`, FR `1 1 0`, RL `1 1 0`, RR `1 -1 0`. **Which physical roller orientation matches this is not verified for the real robot** (Q4 in 01). Wrong roller orientation yields reversed strafe or rotation.

## 3. Kinematics (standard X-layout, verify against the chosen controller)
With L = lx + ly, wheel angular velocities w (rad/s), body velocity (vx, vy, wz):
```
w_fl = ( vx - vy - L*wz) / r
w_fr = ( vx + vy + L*wz) / r
w_rl = ( vx + vy - L*wz) / r
w_rr = ( vx - vy + L*wz) / r
```
Forward (odometry): vx = r/4 (w_fl+w_fr+w_rl+w_rr); vy = r/4 (-w_fl+w_fr+w_rl-w_rr); wz = r/(4L) (-w_fl+w_fr-w_rl+w_rr).
Check the sign of every row by unit test and by simulation (forward, strafe-left, rotate-CCW), not by trusting this table. The "O" layout flips the sign of vy and wz terms.

## 4. Controller options
| Option | Status on this machine | Notes |
|---|---|---|
| Gazebo `MecanumDrive` system plugin (current) | `libignition-gazebo6-mecanum-drive-system.so` present | Takes `/cmd_vel`, publishes odom/TF itself; params in `mecanum_forklift.gazebo.xacro` (`wheel_separation`=2*wheel_y, `wheelbase`=2*wheel_x, `wheel_radius`, accel limits). Semantics of `wheel_separation`/`wheelbase` (full vs half distances) **not verified**. Sim-only; no hardware path. |
| `mecanum_drive_controller/MecanumDriveController` | In `ros-humble-mecanum-drive-controller` (part of ros2_controllers 2.52.0), installed; **chainable controller**, `use_stamped_vel` param present | Needs ros2_control + a hardware interface (`ign_ros2_control` in sim). Parameters seen in installed header: 4 x `*_wheel_command_joint_name` / `*_wheel_state_joint_name`, `kinematics.*` (wheel radius, center projection sum), `reference_timeout`, `enable_odom_tf`, `odom_frame_id`, `base_frame_id`, covariance diagonals. Exact command/state interface names and reference topic: **verify from package docs/source before use** (do not assume `velocity`). |
| Custom controller/node | not started | Only if the stock controller cannot meet a requirement (document why in an ADR). |

Selection is ADR-0002 (undecided). Recommended evaluation order: (1) validate the existing Gazebo plugin as the sim baseline; (2) evaluate `mecanum_drive_controller` + `ign_ros2_control` in a branch of the workspace once install is approved, because it is the only path that shares code with real hardware.

## 5. Interfaces
- Input: `geometry_msgs/Twist` on `cmd_vel` (final arbitrated command, see 05). Controller may use TwistStamped.
- Output: wheel velocity commands; wheel joint states on `/joint_states`; `/odom` (`nav_msgs/Odometry`).
- Limits: vx, vy, wz, accelerations - configured in YAML, applied before the drive (velocity smoother/arbitration), not hard-coded.

## 6. Tests (acceptance for M5)
Automated: kinematics unit test (forward + inverse round trip, signs, zero input) if custom code exists.
Simulation (record numbers in `docs/testing/`):
1. Forward 0.2 m/s for 5 s -> displacement ~1.0 m, |y drift| and |yaw drift| within agreed tolerance.
2. Strafe left 0.2 m/s for 5 s -> y ~ +1.0 m, x/yaw drift small.
3. Rotate +0.5 rad/s for 6 s -> yaw ~ +3.0 rad, position drift small.
4. Combined diagonal + rotation; stop on zero command; stop on command timeout.
5. Compare `/odom` to Gazebo ground truth (pose from `ign topic` or OdometryPublisher).
Tolerances: **TBD, propose and get approval** (suggest <=5% distance, <=3 deg yaw as a starting point).

## 7. Failure modes
Reversed strafe (roller/sign mismatch); sideways drift while driving straight (friction anisotropy, wheel mass/inertia asymmetry); no strafe at all (mesh collision, `mu2` nonzero, `fdir1` lost in SDF); odometry yaw scale wrong (wrong L); slip under acceleration (accel limit); controller not activating (interface names, unconfigured hardware).

## 8. Documentation requirements
Wheel table (name, position, roller direction, joint, sign), the kinematics matrix actually used, controller YAML, test results with date.

## 9. Open
Real roller orientation; real motor/encoder interface; max speeds and accelerations; which controller (ADR-0002).
