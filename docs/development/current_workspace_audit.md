# Current workspace audit (2026-11-05)

Read-only audit. Evidence and commands: `docs/testing/existing_package_test_report.md`; structure: `docs/architecture/existing_workspace_architecture.md`. **Confirmed** = observed on this machine this session; **Assumption** = read from code or inferred, not observed.

## 1. Existing working features (confirmed)
- Environment matches target: Ubuntu 22.04.5, Humble, Ignition Gazebo 6.18.0 (Fortress), ros_gz 0.244.24.
- Xacro expands, `check_urdf` valid, URDF->SDF conversion valid; all 3 packages build; 4 lint tests pass.
- Gazebo launch spawns the robot in `simple_warehouse`; LiDAR (`/scan`, `lidar_link`) and camera (`/camera/image_raw`, `front_camera_link`) publish; `/joint_states` carries 4 wheels + fork.
- TF robot tree connected (`base_footprint -> base_link -> wheels/fork/camera/lidar`).
- Holonomic motion via the Gazebo `MecanumDrive` plugin: backward, strafe left/right, rotate CW/CCW and diagonal all moved in the correct direction with the expected body axes (single trials).
- `ps4_node` maps fake Joy input to correct signs (left stick up/left = +vx/+vy, right stick right = -wz) and drives the fork.

## 2. Errors, gaps and incorrect configuration

| ID | Severity | Finding | Status |
|---|---|---|---|
| A1 | High | **No `/odom` and no `odom->base_footprint` TF.** Cause: Fortress 6.18 `MecanumDrive` never publishes odometry (no `DiffDriveOdometry::Update` call). **FIXED 2026-11-05** with the `OdometryPublisher` system (ground-truth odom, sim only); `/odom` 34 Hz wall, TF connected. | fixed, tested |
| A2 | High | **No velocity watchdog**: the plugin keeps the last `cmd_vel` forever; `ps4_node` replayed the last Joy. **FIXED 2026-11-05**: `cmd_vel_gate` (0.5 s timeout, limits) and `ps4_node` `joy_timeout`. | fixed, tested |
| A3 | Medium | `ros2_control.xacro`, `gazebo.xacro`, `material.xacro` are empty; there is no ros2_control or controller config. The control stack is the Gazebo plugin only, so requirement "ros2_control configuration valid" cannot be assessed (nothing to validate). ADR-0002 open. | confirmed |
| A4 | Medium | `fork_joint` sits at -0.001 m (below lower limit 0) at rest; PID gains (1000/20/50) untested for lift and load. | confirmed value, cause unverified |
| A5 | Medium | Wheel roller orientation is encoded only through `fdir1` and a launch-time string patch (`ignition:expressed_in`) in `make_spawn_node`. Works in sim but is fragile (regex on `ign sdf` output, tmp dirs left in `/tmp`, needs `ign` CLI at launch). Not reusable on hardware. | confirmed by code |
| A6 | Medium | Real-time factor about 0.39 with GUI: sensor rates (scan 4.4 Hz, camera 9.2 Hz wall) are below nominal; cause (software rendering, 1 ms step, GUI) not isolated. | confirmed RTF, cause unverified |
| A7 | Low | FIXED (stage 1): `mecanum_forklift_teleop/package.xml` lacked `<buildtool_depend>ament_python</buildtool_depend>`; no keyboard launch file; no tests beyond lint. | confirmed |
| A8 | Low | `use_sim_time` and `gui` launch args added to the Gazebo launch (defaults true/true). `display.launch.py` still lacks `use_sim_time` (not needed for static display). | fixed (gazebo), tested |
| A9 | Low | Bridge constants migrated to `GZ_TO_ROS`/`ROS_TO_GZ`; the Gazebo `/tf` is now needed (carries `odom->base_footprint`). | fixed, tested |
| A10 | Low | LiDAR `<sensor><pose>` (z +0.03) is not in the TF; camera frame is a Gazebo (x-forward) frame, not a REP-103 optical frame; camera rides on the fork (moves with lift). | assumption/design note |
| A11 | Low | FIXED: unused ros_gz deps removed from `description/package.xml`. `gazebo` package still does not declare the `ign` CLI. | partly fixed |
| A12 | Info | Keyboard node untested (needs TTY); PS4 hardware absent; deadman release, stale Joy untested. | unverified |

No Humble/Fortress incompatibility found: plugin names (`ignition-gazebo-*`), `ign sdf`, ros_gz Humble bridge types all valid. Dependencies for the existing stack are all installed; only the ADR-0002 option B (ros2_control in Gazebo) lacks `ign_ros2_control`.

## 3. Recommended fixes (proposed, not applied)
1. **A1**: inspect `ign topic -e` for `/odom` and `/model/.../odometry`, check plugin frame/topic parameters for the Fortress 6.18 MecanumDrive (`frame_id`, `child_frame_id`, `odom_topic`, `tf_topic`) and whether odometry is gated on commands; fix the bridge/plugin config; decide single owner for `odom->base_footprint` (plugin now, EKF later, never both).
2. **A2**: add a cmd_vel watchdog (timeout -> zero) in the future velocity-arbitration node and a Joy-timeout in `ps4_node`; keyboard: stop on release/timeout option.
3. **A3/ADR-0002**: keep the plugin baseline (as ADR-0002 recommends); delete or fill the empty xacro files only after approval; evaluate `mecanum_drive_controller` after installing `ign_ros2_control` (needs approval).
4. **A4**: check fork collision/inertia/gains in a lift-only test.
5. **A5**: move wheel friction to a pre-generated SDF/model or a documented plugin-free approach so spawn does not rewrite SDF at runtime.
6. **A6-A9, A11**: headless/`use_sim_time` launch args, `rviz` display test, package.xml fixes, bridge constants.

## 4. Development plan (needs your approval before any change)

| Stage | Goal | Acceptance (measurable) |
|---|---|---|
| 1 Description | fix A4, A10, A11, empty xacros, add camera optical frame; verify xacro/check_urdf, `display.launch.py` in RViz | no warnings; wheel joint axes/sign table documented |
| 2 Gazebo integration | headless/GUI args, `use_sim_time` arg, resolve A1 and A6, bridge tidy | `/odom` >= 20 Hz sim-time, `odom->base_footprint` in TF, scan 10 Hz / camera 30 Hz in sim time |
| 3 Mecanum control | keep plugin baseline; add watchdog; (optional, approved) prototype ros2_control branch | each of fwd/back/left/right/rot/combined within +-10 % of commanded distance/angle from a reset pose; stop within 0.5 s of command loss |
| 4 Teleop | keyboard + PS4 tests (fake Joy automated, real hardware manual), Joy timeout, config for speed limits | node tests pass; deadman release stops robot |
| 5 Odometry test | repeatable test script: drive a pattern, compare `/odom` to ground truth | drift per metre / per rotation recorded; strafe and rotate tests mandatory |

Test plan = the commands in the test report, scripted (pose reset between runs, fixed RTF, repeat 3x).

## Open questions for you
1. Keep the Gazebo plugin as the baseline (ADR-0002 option A) for now, or approve installing `ign_ros2_control` to prototype option B?
2. May I start Stage 1-2 (fix `/odom`, edit description/gazebo files) in the existing packages?
3. Is the GUI-on default acceptable, or do you want headless as default?

## 5. Stage 1-2 progress (2026-11-05, approved: stages 1-2, plugin baseline, GUI default)

| Item | Status |
|---|---|
| A1 odometry (`OdometryPublisher`, sim ground truth, single owner of `odom->base_footprint`) | tested |
| Launch args `gui` (default true) and `use_sim_time`; bridge constants; package.xml fixes | tested |
| Build + 8 lint tests; 6 motion types + combined, correct axes, no cross-coupling | tested |
| A4 fork -0.001 m sag, A6 low RTF (0.4-0.7), A10 camera optical frame, A5 spawn-time SDF patch, empty xacro files | open (not touched) |
| Hardware odometry drift | unverified (needs hardware) |
| Watchdog for `cmd_vel` and Joy (`mecanum_forklift_control/cmd_vel_gate`, `ps4_node joy_timeout`) | tested |
| Wheel odometry (`mecanum_forklift_control/wheel_odometry`; owner of `/odom` + TF; ground truth on `/odom_ground_truth`) | tested, matches ground truth within 0.2 mm in sim |

Note: `/odom` is **ground truth**, so odometry-quality tests are meaningless until wheel odometry exists. Decision to confirm: add a wheel-odometry node (forward kinematics from `/joint_states`) in stage 5 and then switch the owner of `odom->base_footprint` (never both).
