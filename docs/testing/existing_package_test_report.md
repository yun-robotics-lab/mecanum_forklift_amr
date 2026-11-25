# Existing package test report (2026-11-05)

Scope: read-only audit of `mecanum_forklift_description`, `_gazebo`, `_teleop`. Nothing in `src/`, `build/`, `install/`, `log/` was modified. Build, tests and simulation ran from a scratch build (`--build-base/--install-base` in the session scratchpad) with the workspace `install/` untouched (its `gazebo.xacro` copy is identical to `src/`). Status words: implemented / built / tested / verified / blocked / unverified. The sim ran **with the GUI** (DISPLAY=:0), real-time factor about 0.39.

## Environment (all confirmed)
Ubuntu 22.04.5; `ROS_DISTRO=humble`; Ignition Gazebo 6.18.0 (`ign gazebo --versions`), `ignition-fortress 1.0.3`; ros_gz 0.244.24 (`ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, `ros_ign_bridge`); ros2_controllers 2.52.0 incl. `mecanum_drive_controller`; controller_manager 2.53.1; xacro 2.1.1; robot_state_publisher 3.0.3; rviz2; joy 3.3.0; teleop_twist_joy/keyboard; robot_localization 3.5.4; slam_toolbox 2.6.10; nav2_bringup 1.1.20. Not installed: `ros2_control` meta-package, `ign_ros2_control`/`gz_ros2_control`. No joystick device (`/dev/input/js*` absent).

## Results

| # | Check | Command | Result | Status |
|---|---|---|---|---|
| 1 | Xacro expands | `xacro src/mecanum_forklift_description/urdf/mecanum_forklift.urdf.xacro > robot.urdf` | rc 0, no warnings | tested |
| 2 | URDF valid | `check_urdf robot.urdf` | "Valid", tree as documented in architecture file | tested |
| 3 | URDF->SDF | `ign sdf -k robot.urdf` and `ign sdf -p` | valid; 4 `<fdir1>` present; mesh URIs `model://mecanum_forklift_description/...` | tested |
| 4 | Build (3 pkgs) | `colcon build` (scratch dirs) | 3 packages finished | built |
| 5 | Package tests | `colcon test` | 4 tests, 0 errors, 0 failures | tested (lint only) |
| 6 | Sim launch | `ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py` | world loads, robot spawns, `MecanumDrive subscribing to twist messages on [cmd_vel]`; only warnings: bridge `IGN_TO_ROS` deprecated, ogre2 mask, QML | tested |
| 7 | Topics | `ros2 topic list` | `/scan /camera/image_raw /camera/camera_info /clock /cmd_vel /fork/cmd_pos /joint_states /odom /tf /tf_static /robot_description` present | tested |
| 8 | LiDAR | `topic echo /scan --once`; `topic hz` | frame `lidar_link`, range 0.12-8.0, 4.4 Hz wall (RTF 0.39) | tested |
| 9 | Camera | same | frame `front_camera_link`, 640x480 rgb8, 9.2 Hz wall | tested |
| 10 | Joint states | echo | 7 joints incl. 4 wheels + `fork_joint` (fork at -0.001, below its 0.0 lower limit) | tested |
| 11 | TF tree | `ros2 run tf2_tools view_frames` | connected `base_footprint -> base_link -> {4 wheels, fork_link -> front_camera_link, lidar_link}`; **no `odom` frame** | tested |
| 12 | `/odom` | `topic echo /odom --once`, `ign topic -e -t /odom` (while commanding) | **no message received** (Gazebo-side publisher exists, nothing emitted within 3-5 s) | tested - FAIL |
| 13 | ros2_control | n/a | `ros2_control.xacro` empty, no controller YAML, no controller_manager launched | not implemented |
| 14 | PS4 node | fake `/joy` (axes LX=1, LY=1, RX=-1, L1+Triangle held) | `/cmd_vel` = (0.3, 0.3, 0, ang z -1.0); `/fork/cmd_pos` ramped to 0.12 | tested (no hardware) |
| 15 | Keyboard node | needs a TTY | not run | unverified |

### Motion tests (ground truth = Gazebo `dynamic_pose/info`, body frame, 4 s wall = about 1.6 s sim)

| Motion | Command | Ground-truth result | Verdict |
|---|---|---|---|
| backward | vx -0.2 | dx -0.235, dy 0, dyaw 0 | moves correctly |
| strafe left | vy +0.2 | dy +0.311, dx 0, dyaw 0 | correct |
| strafe right | vy -0.2 | dy -0.315, dx 0, dyaw 0 | correct |
| rotate CCW | wz +0.5 | dyaw +44.9 deg (expected 45 at 1.57 s sim) | correct |
| rotate CW | wz -0.5 | dyaw -44.7 deg | correct |
| diagonal | vx 0.15, vy 0.15 | dx +0.215, dy +0.215 | correct, also shows +x works |
| combined | vx .15, vy .1, wz .4 | small, inconsistent displacement | **inconclusive** (robot was likely in contact with a wall after earlier runs) |
| forward (isolated) | vx +0.2 | 0.000 / later 0.05 with yaw drift | **inconclusive**: robot was against walls/obstacles; forward only confirmed through the diagonal case |

Honest limits: single trials, timings dependent on RTF, no repeat after a pose reset, no `/odom` to compare against. Speeds were measured only as order of magnitude (about 0.6-0.8 of commanded distance in the available window; acceleration limit 2 m/s^2 and sim startup not separated). Treat as "direction and axis sign correct", not as accuracy.

### Observed anomaly
After one `ros2 topic pub -r 20 /cmd_vel` of 0.2 m/s for 6 s wall time, the model later read x = 1.77 m (expected about 0.5 m). Likely cause (**unverified**): the Fortress `MecanumDrive` plugin keeps the last Twist and has no command timeout, so the robot kept driving after the publisher stopped. This matches the walls later blocking forward motion. Needs a dedicated test (publish one Twist, stop, watch pose).

## Not tested / unverified
- Wheel roller handedness visually (single wheel mesh reused for all four; only friction directions encode the X pattern; behaviour tests pass).
- Real PS4 hardware, axis/button indices (assumed DualShock 4 hid-playstation layout), deadman release and stale-Joy behaviour.
- Keyboard node; RViz display launch (`display.launch.py`); `simple_warehouse` model placement vs collisions; second world `empty.sdf` launch.
- Sensor rates in sim time at RTF 1.0; lidar/camera data content (only headers/metadata checked).
- Physical safety: simulation does not validate it.

## Reproduce
```bash
cd ~/Personal_Project/mecanum_forklift_amr && source /opt/ros/humble/setup.bash
xacro src/mecanum_forklift_description/urdf/mecanum_forklift.urdf.xacro > /tmp/robot.urdf && check_urdf /tmp/robot.urdf
colcon build && colcon test && colcon test-result --verbose     # use your normal build; the audit used scratch dirs
source install/setup.bash && ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py
# other terminal
ros2 topic hz /scan; ros2 topic echo /odom --once            # /odom never arrives
ros2 run tf2_ros tf2_echo odom base_footprint                # frame does not exist
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.2}}"   # strafe left
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"  # rotate; then send zero Twist
```

---

# Stage 1-2 changes: results (2026-11-05, after approval)

Changes tested: `OdometryPublisher` added, ignored odometry params removed from `MecanumDrive`, launch args `gui` (default true) and `use_sim_time` (default true), bridge constants `GZ_TO_ROS`/`ROS_TO_GZ`, `package.xml` fixes. Run in the real workspace (`colcon build`), sim headless (`gui:=false`), RTF 0.66-0.71.

| Check | Result | Status |
|---|---|---|
| Root cause of missing `/odom` | `nm -DC libignition-gazebo-mecanum-drive-system.so`: only `DiffDriveOdometry` ctor/`SetWheelParams` linked; `Init`/`Update` absent (present in the diff-drive plugin). Fortress 6.18 `MecanumDrive` never publishes odometry; its `odom_topic`/`tf_topic` params are ignored | confirmed (binary symbols) |
| Build + lint | `colcon build` 3 packages; `colcon test` 8 tests, 0 errors/failures; xacro + `check_urdf` valid | built, tested |
| `gui:=false` | only `ign gazebo -s` server started, sim and bridge came up, no errors | tested |
| `/odom` | `frame_id: odom`, `child_frame_id: base_footprint`, 34 Hz wall (about 51 Hz sim time) | tested |
| TF | `view_frames`: `odom -> base_footprint -> base_link -> {wheels, fork_link -> front_camera_link, lidar_link}`; `tf2_echo odom base_footprint` works | tested |
| Sensor rates (wall, RTF 0.66) | `/scan` 6.8 Hz (10 nominal), `/camera/image_raw` 18.1 Hz (30 nominal); roughly nominal in sim time but not measured in sim time | tested (partly) |
| Steady-state twist from `/odom` | commanded (0.3, 0.3, -1.0) gave (0.2997, 0.3003, -1.0): scale within 0.1 % | tested |
| Motion, pose reset before each run (GT = odom) | forward / backward / strafe L / strafe R / rotate CCW / CW / diagonal / combined: all in the correct axis; forward and backward had dy = yaw = 0, strafes dx = yaw = 0, rotations no translation; diagonal dx = dy; combined has +x, +y, +yaw | tested |
| Distances | 67-78 % of ideal over "3 s sim": **not reliable**, timing used whole-second sim stamps (+-1 s); use the twist check above | unverified |
| `/odom` accuracy vs wheel odometry | `/odom` is ground truth, so it equals Gazebo pose by construction; wheel odometry/drift not yet implemented | not applicable |

## Newly confirmed
- **Plugin latches the last Twist** (A2): with zero `/cmd_vel` publishers the robot kept moving at the last command. Needs a watchdog outside the plugin.
- **Stale-Joy hazard** (A2): `ps4_node` kept republishing the last Joy-based command at 20 Hz after its `/joy` publisher ended, until killed. This also caused an apparent "drift" while testing (my own stray test process, since stopped).
- The earlier "1.77 m" anomaly is explained by the latching plugin, not by physics.
- Cleanup note: killing the `ros2 run` wrapper does not stop the node; stop the real PID.

---

# Stage 3-4: cmd_vel gate (watchdog) and wheel odometry (2026-11-05)

New package `mecanum_forklift_control` (approved): `cmd_vel_gate` (`cmd_vel_teleop` -> `cmd_vel`, wall-clock timeout 0.5 s, limits 0.6/0.6/2.0, rejects non-finite) and `wheel_odometry` (forward kinematics from `/joint_states`; owner of `/odom` and `odom->base_footprint`). Teleop nodes now publish `cmd_vel_teleop`; `ps4_node` has `joy_timeout` 0.5 s. Gazebo ground truth moved to `/odom_ground_truth` (its TF is not bridged). Sim run headless in an isolated `ROS_DOMAIN_ID=42`, RTF 0.4-0.7.

| Check | Result | Status |
|---|---|---|
| Build, 4 packages | ok | built |
| `colcon test` | 16 tests, 0 failures (6 new unit tests for kinematics and gate logic, plus flake8/pep257) | tested |
| Gate idle | `/cmd_vel` = zero with no teleop input | tested |
| Limit clamp | sent (5, -5, 9) -> `/cmd_vel` (0.6, -0.6, 2.0) | tested |
| Watchdog | driving at 0.3 m/s, then input silent: `/cmd_vel` zero after 0.48 s; ground-truth robot stopped after 0.69 s | tested |
| Stale Joy (`ps4_node`) | `/joy` stopped: "no /joy for 0.5 s: stopping", one zero Twist, robot twist 0 (before the fix it replayed the last command until killed) | tested |
| Frame ownership | `/odom` has one publisher (`wheel_odometry`), `/odom_ground_truth` one (bridge), `odom->base_footprint` from `wheel_odometry` | tested |

## Wheel odometry vs ground truth (5 s sim per motion, pose reset, body-frame deltas, Gazebo `/odom_ground_truth` as reference)

| Motion | Ground truth (x m, y m, yaw deg) | Wheel odom error (mm, mm, deg) |
|---|---|---|
| forward 0.2 m/s | +1.000, 0, 0 | 0.0, 0.0, 0.00 |
| backward | -1.000, 0, 0 | 0.0, 0.0, 0.00 |
| strafe left / right | 0, +1.000 / -1.000, 0 | 0.0, 0.0, 0.00 |
| rotate CCW / CW 0.5 rad/s | 0, 0, +143.2 / -143.3 | 0.0, 0.0, 0.00 |
| diagonal 0.15, 0.15 | +0.750, +0.750, 0 | 0.0, 0.0, 0.00 |
| combined 0.15, 0.1, 0.4 | +0.015, +0.755, +113.4 | -0.1, 0.0, 0.00 |
| circle 0.2, wz 0.5 | +0.266, +0.711, +143.2 | -0.2, +0.1, 0.00 |

Distances equal command times duration exactly (1.000 m = 0.2 m/s x 5 s), so the earlier "67-78 %" figures were a timing artefact. The sim has **no wheel slip**, so near-zero error says the kinematics and signs are right, not that the real robot's odometry will be this accurate. Hardware drift is unverified.

## Limits and notes
- Wheel odometry is not reset when the pose is teleported (`set_pose`); compare deltas. A reset service is not implemented.
- The gate publishes zero continuously; any other direct publisher on `/cmd_vel` would fight with it (by design, `cmd_vel` has a single owner).
- Keyboard node: now publishes `cmd_vel_teleop`; the keys were tried by the user, not by me (unverified by me). It stays latched (no key timeout); the gate only protects against node death.
- An older keyboard node started before the rebuild kept publishing to `/cmd_vel` and masked the gate; restart teleop nodes after rebuilding.
- Max speed limits are sim placeholders, not hardware values. Simulation does not validate physical safety.
