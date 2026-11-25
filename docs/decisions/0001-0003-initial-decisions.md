# Initial architecture decisions

## ADR-0001 - Target environment fixed (accepted)
- **Decision**: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress.
- **Reason**: project requirement; the reference repo and local installation target the same stack.
- **Alternatives**: Ubuntu 24.04 + Jazzy + Harmonic - rejected by the owner.
- **Consequences**: use `ign` CLI and `ignition-gazebo-*` plugins; only Humble-era package versions and docs; some newer Nav2/ros2_control features unavailable.
- **Verification**: per-dependency checks against the local installation before use.

## ADR-0002 - Drive/control stack (ACCEPTED for simulation, 2026-11-06)
- **Decision**: simulation uses the Gazebo `MecanumDrive` system plugin fed by `cmd_vel_gate` (`cmd_vel_teleop` -> `cmd_vel`), with wheel odometry from `mecanum_forklift_control`. No `ros2_control` / `ign_ros2_control` for now.
- **Reason**: works today; strafe/rotate/combined tests pass (docs/testing); no new dependencies. Adopting `ros2_control` would need a system install (`ign_ros2_control` 0.7.21 candidate, controller_manager 2.53.1 installed vs 2.54.3 candidate = version skew risk).
- **Alternative (B)**: `mecanum_drive_controller` + `ign_ros2_control`, same controller on sim and hardware. Rejected for now, not discarded.
- **Consequences**: the plugin is sim-only. Hardware gets its own ADR in M16 (likely B or a custom driver node behind the same `cmd_vel` interface). Keep `cmd_vel` as the stable boundary so the swap does not touch teleop, Nav2 or missions.
- **Verification**: strafe + rotate tests on A recorded in docs/testing; interface names of B unverified.
- **History**: previously PROPOSED (recommend A through M5, prototype B on approval).

## ADR-0003 - Reference repository usage (accepted)
- **Decision**: use `boyloy21/autonomous-mini-forklift` as architectural reference only; keep it unmodified; inspect before reusing; read from an external clone.
- **Reason**: it is a differential-drive system (drive, odometry, MPPI `DiffDrive`, AMCL differential model, ros2_control diff-drive controller are not applicable).
- **Likely reusable concepts**: interface layout (tasks, missions, lift), lift controller split, docking with dock database, task/mission managers, web bridge and ports, bringup staging, route graph.
- **Verification needed**: per-package review in M1 and Humble-compatibility check of each dependency.

## ADR-0004 - TF ownership and odometry source (ACCEPTED for simulation, 2026-11-06)
- **Decision**: `odom -> base_footprint` is owned by `wheel_odometry` (package `mecanum_forklift_control`, 50 Hz, `publish_tf: true`); `base_footprint -> base_link` and all fixed/joint links by `robot_state_publisher`; `map -> odom` by localization (SLAM Toolbox while mapping, AMCL while navigating; never both) once those exist. No EKF.
- **Reason**: single publisher per child frame verified (`/tf` publishers: `robot_state_publisher`, `wheel_odometry`; `/tf_static`: `robot_state_publisher`); Gazebo ground-truth TF goes to `/ground_truth/tf`, which is not bridged. Wheel odometry matched ground truth to <= 0.1 mm over all motions; a `robot_localization` EKF (odom vx/vy/wz + IMU yaw rate) gave no measurable improvement.
- **Alternatives**: EKF owns `odom -> base_footprint` (would require `publish_tf: false` on `wheel_odometry`). Not adopted; revisit with hardware data.
- **Consequences**: the sim cannot size covariances (error ~0, no wheel slip in the sim friction model). Pose/twist variances (0.01) remain placeholders. Hardware drift is unverified.
- **Verification**: docs/testing/2026-11-06_m6_odometry_tf.md.

## ADR-0005 - Mapping setup (ACCEPTED for simulation, 2026-11-06)
- **Decision**: new package `mecanum_forklift_localization` (config, launch, maps; AMCL joins it in M8). SLAM Toolbox online async, resolution 0.05 m, `max_laser_range` 8.0 / `min_laser_range` 0.12 (sensor model), `map_update_interval` 2.0, `minimum_travel_distance/heading` 0.3, `enable_interactive_mode` false; otherwise Humble defaults. Maps stored in this package's `maps/` for now (the navigation package does not exist; revisit in M9).
- **Reason**: tests in docs/testing/2026-11-06_m7_slam_mapping.md (pose error <= 12 mm / 0.84 deg, map 5.975 x 5.000 m vs 6.0 x 5.0).
- **Consequences**: dependencies `slam_toolbox`, `nav2_map_server`, `nav2_lifecycle_manager` (installed). Pallet-level objects are invisible to the 0.224 m lidar plane.

## ADR-0006 - AMCL localization setup (ACCEPTED for simulation, 2026-11-06)
- **Decision**: `nav2_amcl::OmniMotionModel`, likelihood-field laser model, `base_footprint`, scan `/scan` (0.12-8.0 m), 1000-5000 particles, `sigma_hit` 0.1, 120 beams, `update_min_d/a` 0.05, `recovery_alpha_*` 0. Launch: `mecanum_forklift_localization/launch/localization.launch.py` (map_server + amcl + lifecycle manager, initial pose via launch arguments). AMCL owns `map->odom` while localizing; never together with slam_toolbox.
- **Reason**: tested in docs/testing/2026-11-06_m8_amcl_localization.md: RMS 33-34 mm / 0.25 deg over the tour with the omni model versus 527 mm / 26 deg with the differential model.
- **Consequences**: the initial pose must be roughly right (<= ~0.5 m / 30 deg); a gross error needs the global-localization service. `recovery_alpha_*` stays 0 (no random-pose injection). Values are tuned on a noiseless sim and must be re-tuned on hardware.

## ADR-0007 - Interfaces package, minimal start (ACCEPTED, 2026-11-14)
- **Decision**: create `mecanum_forklift_interfaces` (ament_cmake) with only `LiftState.msg` and `SetLiftHeight.action`. Codes are `uint8` constants plus a free-text `message`; no string enums. Dock and mission types are added in M11/M12.
- **Reason**: lift is the first consumer (M10); defining dock/mission types now would be guesses. Constants avoid the reference repo's unspecified `task_type` strings.
- **Alternatives**: copy the reference's ~40 messages (rejected, ADR-0003); also define `DockRobot`/`ExecuteTask` now (rejected, user chose lift only).
- **Consequences**: new package; dependency only on `std_msgs`. Contract in docs/architecture/interfaces.md.
- **Verification**: built, `colcon test` (21 lint tests pass), Python serialize round-trip. C++ consumers and action server/client calls untested (none exist yet).

## ADR-0008 - Lift controller (ACCEPTED for simulation, 2026-11-14)
- **Decision**: new package `mecanum_forklift_lift` (ament_python): `lift_controller` node with a ROS-free `LiftCore` state machine (UNINITIALIZED, HOMING, IDLE, MOVING, FAULT), `SetLiftHeight` action (rejections returned as aborts with a code), `lift_state` topic (10 Hz), `lift_reset_fault` (std_srvs/Trigger). It ramps `fork/cmd_pos` at <= 0.05 m/s default (max 0.1). Teleop (PS4 and keyboard) now publishes `fork/cmd_teleop`; the controller clamps it and ignores it during an action or fault. Gazebo launch includes `lift.launch.py`.
- **Reason**: commands must pass limit checks (CLAUDE.md); explicit states and faults per reference 10; core testable without ROS.
- **Plant change**: Gazebo fork PID p 1000/i 20/d 50 -> 5000/50/150; the old gains sagged 5-9 mm and timed out on a fresh start.
- **Alternatives**: ros2_control lift position controller (not installed; ADR-0002), teleop publishing straight to `fork/cmd_pos` (rejected: bypasses limits).
- **Consequences**: dependency on `mecanum_forklift_interfaces`; gazebo package gets an exec dependency on the lift package. Limits and timeouts are sim placeholders.
- **Verification**: docs/testing/2026-11-14_m10_lift_control.md. Not verified: payload, hardware, driving interlock.

## ADR-0009 - Pallet vision and docking (ACCEPTED for simulation, 2026-11-19)
- **Decision**: new packages `mecanum_forklift_vision` (`aruco_node`, OpenCV ArUco, poses via tf2 at the image stamp) and `mecanum_forklift_docking` (`DockPallet` action). Docking = `opennav_docking` 0.0.2 (SimpleChargingDock, external marker pose) for the coarse approach, ending 0.43 m before the marker, then our own holonomic final alignment (vx, vy, wz, `align_core.py`) on the latched marker pose, then `SetLiftHeight`, optional undock. Velocity only on `cmd_vel_dock` (new `twist_mux` input, priority 50 between teleop 100 and navigation 10) and through `cmd_vel_gate`.
- **Reason**: the stock opennav controller outputs vx/wz only; with a lateral or yaw offset it could not stop on the target (1/7 trials, docs/testing/2026-11-15_m11b_docking.md). The camera sits on the fork root and loses the marker closer than ~0.28 m, so the last ~0.2 m uses the last far sighting (static dock in `odom`) plus wheel odometry.
- **Alternatives**: opennav only, with looser tolerances (rejected: fails the +-10 deg cases); fully custom docking server (earlier recommendation; user chose opennav for staging/retry/undock); `aruco_opencv` package (not used, own node avoids the dependency).
- **Consequences**: stop distance (0.245 m: tines 0.133 m into the pallet, past its centre of mass, carriage 7 mm short of the pallet end; 0.278 m tilted the pallet) appears in two places (`external_detection_translation_x` = -(stop + 0.15) and workflow `stop_distance`); internal alignment tolerance is half the acceptance tolerance; `corner_expand_px` 0.75 is a sim-only vision calibration; `cv_bridge` is not used (numpy clash).
- **Verification**: docs/testing/2026-11-15_m11a_vision.md, 2026-11-19_m11b_final_alignment.md. Engagement and undock: docs/testing/2026-11-19_m11b_engagement_undock.md. Not verified: Nav2 staging (M9), payload realism, hardware.

## ADR-0010 - Local controller for navigation (ACCEPTED for simulation, 2026-11-23)
- **Decision**: MPPI with the Omni motion model (`nav2_mppi_controller`) is the default local controller (`controller_type:=mppi`); DWB stays available (`dwb`) with `ObstacleFootprint` critic; progress checker 0.10 m / 30 s in all controller configs; MPPI goal tolerance 0.04 m / 0.07 rad.
- **Reason**: same 10 goals (docs/testing/2026-11-23_m9_navigation.md): MPPI 20/20 succeeded, 17/20 within 5 cm / 5 deg, no contact, strafes in 95-99 % of the sideways goals; DWB 9/10 (a local minimum when turning 180 deg in the cone aisle), centre-only obstacle critic hit cones, strafes in 17-25 %.
- **Alternatives**: DWB (kept as fallback, goal 6 unsolved), RPP / rotation shim (config exists, not tested; no vy).
- **Consequences**: the 5 cm criterion is close to AMCL's own error (3-7 cm); Nav2 has no bounded give-up time for blocked goals, so M12 must time out and cancel; footprint is the unloaded robot incl. tines.
- **Verification**: docs/testing/2026-11-23_m9_navigation.md. Not verified: loaded footprint, collision monitor, hardware, other worlds.

## ADR-0011 - Pick-and-carry integration (ACCEPTED for simulation, 2026-11-24)
- **Decision**: the `DockPallet` workflow owns the whole pick: it navigates to the staging pose itself (Nav2 `NavigateToPose`), waits for a fresh marker pose, then runs opennav (staging disabled), holonomic alignment and lift. After the lift it switches the costmap footprint to the loaded polygon (topic `local/global_costmap/footprint`) and, if the dock entry has `map_after_pick`, loads a curated map without that pallet (`nav_map_server/load_map`); both are reverted when the next dock starts. The vision node respawns itself (launch `respawn=True`) after 10 s without images.
- **Reason**: opennav's own staging would run before the marker can be seen and defeats the stale-pose check; the loaded footprint overlapped the painted slot of the picked pallet, so every loaded plan aborted; the image stall (root cause unknown) made runs unreliable.
- **Alternatives**: opennav does the staging (rejected, see above); leave footprint and map handling to the mission manager (rejected: the pallet state is known here); a generic dynamic obstacle layer instead of map swapping (more general, not needed while pallets are only known from a painted map).
- **Consequences**: the dock database carries `map_after_pick`; `staging_x_offset` exists in two files (docking_server.yaml, workflow.yaml); the docking package depends on the navigation package and `nav2_msgs`.
- **Verification**: docs/testing/2026-11-24_m11c_pick_carry.md (20/20 trials, 3/3 failure cases). Not verified: drop-off, other pallets, hardware.

## ADR-0012 - Mission manager (ACCEPTED for simulation, 2026-11-24)
- **Decision**: new package `mecanum_forklift_task_manager` (ament_python): one `mission_manager` node serving `ExecuteMission` (PALLET_TRANSPORT: pick dock, drop dock, optional return pose) as a client of Nav2 `NavigateToPose` and our `DockPallet`; the logic is a ROS-free state machine (`MissionRunner`) with an executor interface so it is unit-tested with mocks. Policy: 1 retry for STAGING_FAILED / MARKER_NOT_FOUND / NAV_FAILED, timeouts 150 s navigation / 180 s dock / 600 s mission (sim time), one mission at a time (second one aborted with BUSY, no queue), cancel propagated to the child goal with stop confirmation and the fork held, child goals cancelled on node shutdown. Drops are `DockPallet` dock entries with `operation: drop`, placed by map-frame poses (no marker).
- **Reason**: Nav2 has no bounded give-up time, so the manager owns timeouts; one orchestrator calling our workflow avoids the reference repo's duplicated orchestrators; the fork-mounted camera cannot see a drop marker through the carried pallet.
- **Alternatives**: BehaviorTree.CPP / py_trees (no framework needed yet, ADR rule), a two-level mission + task manager as in the reference (one level is enough for one mission type), a marker-based drop (blocked by the load), a mission queue (rejected for now).
- **Consequences**: drop accuracy is AMCL-limited (4-7 cm, criterion revised to "fully on the pad, centre within 8 cm"); no persistence (a drop after a node restart is refused with NOT_CARRYING); a second drop dock or pick dock needs its own entry and curated maps.
- **Verification**: docs/testing/2026-11-24_m12_mission.md (39 unit tests, 11/11 missions, 10/10 failure cases). Not verified: teleop during a mission, other pallets, hardware.

## ADR-0013 - Web layer (ACCEPTED for simulation, 2026-11-24)
- **Decision**: new package `mecanum_forklift_web_bridge`: FastAPI REST API on 8000 (missions, status, docks, SSE events, e-stop engage), a separate latched `estop_node`, rosbridge on 9090 and `web_video_server` on 8080, all on 127.0.0.1 by default. Auth (Q-W1): bearer token from a 0600 file, constant-time compare, CORS origin list, rate limit on mission POST/cancel (not on e-stop). Validation: strict schema (unknown fields rejected), known docks with the right operation, return pose inside the map, 4 KB body limit, duplicate ids, robot-state checks (e-stop 423, lift fault, busy). No endpoint accepts velocity or lift commands. E-stop: new `twist_mux` input `cmd_vel_estop` (priority 200) held at zero + cancel of all goals; release only through the local ROS service, never over REST or rosbridge. Minimal static dashboard served by the API.
- **Reason**: roadmap safety rules (web input must never bypass arbitration or limits; e-stop independent of the mission layer and not remotely disable-able); rosbridge 2.0.4 has no authentication and a single shared glob for subscribe/advertise/publish, so it is restricted to telemetry and localhost and the REST event stream is the remote-safe path; the reference repo's REST layer (0.0.0.0, CORS *, no auth) was not copied.
- **Alternatives**: rosbridge as the main command path (rejected: no auth, no validation), WebSocket from the API instead of SSE (SSE is simpler and enough at 2 Hz), a Node/React dashboard (needs a build chain), WebRTC video (heavier than MJPEG for the CPU-limited sim).
- **Consequences**: fastapi and uvicorn are pip dependencies (documented; apt versions too old); a software e-stop is not a safety function; missions are refused when the e-stop state is unknown; rosapi is not started so roslibjs introspection does not work.
- **Verification**: docs/testing/2026-11-24_m13_web.md (64 unit tests, 9/9 integration cases). Not verified: remote access/TLS, browser UI, multi-robot ports, hardware.
