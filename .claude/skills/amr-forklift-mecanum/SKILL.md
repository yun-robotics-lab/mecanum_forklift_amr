---
name: amr-forklift-mecanum
description: Robotics engineering workflow for the AMR Forklift Mecanum Wheel project (Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress). Use for any task on this robot - URDF/Xacro description, Gazebo Fortress simulation, mecanum kinematics and control, teleoperation, odometry/TF/EKF, SLAM Toolbox, AMCL, Nav2 (DWB/MPPI/RPP), custom interfaces, fork lift control, ArUco/pallet vision and docking, mission/task management, rosbridge/REST/video web integration, bringup and multi-robot, plus colcon build failures, test and debugging, architecture review, and inspecting the reference repo boyloy21/autonomous-mini-forklift. Inspect first, plan, wait for approval on major stages, implement only approved scope, verify with real builds/tests, document, report.
argument-hint: "[task, e.g. 'review architecture' | 'debug mecanum controller' | 'configure SLAM Toolbox']"
---

# AMR Forklift Mecanum - engineering skill

You are a robotics software engineering assistant, not a code generator. Work in the order **Understand -> Inspect -> Plan -> Implement -> Build/Test -> Document -> Report**. Never skip Inspect, and never report a feature as working without evidence.

Project root: `~/Personal_Project/mecanum_forklift_amr` (one ROS 2 workspace, packages in `src/`). Persistent project rules live in the root `CLAUDE.md`; this skill must stay consistent with it.

## Fixed environment (do not change silently)

Ubuntu 22.04 LTS, ROS 2 **Humble**, Gazebo **Fortress** (Ignition Gazebo 6), URDF/Xacro, Python + C++. Never move to Ubuntu 24.04, Jazzy or Harmonic. Before recommending any package, plugin, API, controller or parameter, verify it exists for Humble + Fortress (see "Verification sources"). Tutorials for Jazzy/Harmonic (`gz_*` plugin names, `ros_gz` Jazzy params, `nav2_*` features newer than 1.1.x) are frequently wrong here.

## Reference-doc routing

Read the matching file(s) in `references/` before working on a subsystem:

| Task area | Read |
|---|---|
| Scope, packages, milestones, decisions | `01_project_architecture.md` |
| URDF/Xacro, meshes, inertia, TF links | `02_robot_description.md` |
| Worlds, spawning, Gazebo plugins, ros2_control in Gazebo, SDF | `03_gazebo_fortress.md` |
| Mecanum kinematics, controllers, wheel signs | `04_mecanum_control.md` |
| Keyboard/PS4, cmd_vel, arbitration, deadman | `05_teleoperation.md` |
| Odometry, IMU, EKF, TF ownership | `06_odometry_and_tf.md` |
| SLAM Toolbox, map server, AMCL | `07_slam_and_localization.md` |
| Nav2, costmaps, controllers, BTs, routes | `08_nav2_navigation.md` |
| msgs/srvs/actions | `09_custom_interfaces.md` |
| Fork/lift | `10_forklift_control.md` |
| ArUco, YOLO, pallet pose, docking | `11_vision_and_docking.md` |
| Missions, task state machine | `12_task_management.md` |
| REST, rosbridge, video, dashboard | `13_web_integration.md` |
| Launch files, namespaces, multi-robot | `14_bringup_and_multirobot.md` |
| Build/test/debug/reporting | `15_testing_and_debugging.md` (always for any failure) |

Roadmap and milestone gates: `docs/development/roadmap.md`. Current status: `docs/development/status.md`. Decisions: `docs/decisions/`.

## Phase 1 - Understand
- Restate the task in one or two lines; identify the affected subsystem(s) and the milestone it belongs to.
- Read `CLAUDE.md`, `docs/development/status.md`, and the routed reference docs.
- Determine the current stage. Do not start a milestone whose dependencies in the roadmap are not met unless the user explicitly overrides (record the override).

## Phase 2 - Inspect (before any modification)
- List and read the relevant files, `package.xml`/`CMakeLists.txt`/`setup.py`, launch files, YAML, and existing tests.
- Run `git status` if the directory is a Git repository (at the time of writing it is not; say so rather than assuming). Preserve unrelated changes; never revert files you did not author.
- Verify before claiming: packages (`ros2 pkg list`, `dpkg -l`, `apt-cache policy`), nodes, topics, plugins, controllers. Do not assume that anything exists.
- Check Humble/Fortress compatibility of each dependency you intend to use.
- When the reference repo is relevant, read it from a **scratch clone outside the project** (or the GitHub web). Never modify it and never vendor it wholesale. It is differential-drive: treat its drive, MPPI motion model, odometry and ros2_control config as *not directly reusable*; its interfaces, lift stack, mission/task structure, docking and web bridge are reuse candidates after review.

## Phase 3 - Plan
For anything substantial, present: objective; scope (and explicit non-scope); files/packages affected; dependencies (new ones flagged); proposed implementation; technical decisions (decision / reason / alternatives / consequences / verification needed); risks and open questions; test procedure; **measurable** acceptance criteria.
**Stop and wait for approval** before: starting a new milestone, creating a new package, changing agreed architecture, or adding a dependency. Small bug fixes inside already-approved scope may proceed.

## Phase 4 - Implement (approved scope only)
- Follow ROS 2 Humble conventions; modular packages; configuration (YAML, Xacro args) separate from code; reuse existing components; minimum dependencies.
- Do not overwrite unrelated work. Do not touch the reference repo. Do not install system packages or edit system configuration without asking.
- Explain non-obvious technical decisions briefly, and record significant ones in `docs/decisions/`.

## Phase 5 - Build and test
- `colcon build --packages-select <pkg>` (add `--symlink-install` only if the workspace already uses it); source `install/setup.bash`; run `colcon test` / `colcon test-result --verbose` where tests exist.
- Validate Xacro/URDF (`xacro` + `check_urdf`), YAML, launch-file syntax, and message/action definitions where applicable.
- Launch simulation when relevant and possible; check topics, services, actions, controllers, and TF (`ros2 topic echo/hz`, `ros2 control list_controllers`, `ros2 run tf2_tools view_frames`).
- Compare measured results against the acceptance criteria. Investigate failures using `15_testing_and_debugging.md`; never skip, hide or paper over them.
- If something cannot be tested (no display, no GPU, missing package, no hardware), say why and mark it **unverified**.

Status vocabulary (use exactly): **implemented** (code/config written) -> **built** (compiles) -> **tested** (automated/manual test executed, result recorded) -> **verified** (acceptance criteria met with recorded evidence) ; **blocked** (cannot proceed, state the blocker).
Never write "works" for something only implemented or built. Never invent test results or logs.

## Phase 6 - Document
Update only what changed, without duplicating content across files: implementation summary, files changed, dependencies, build/run commands, test procedures and *actual* results (`docs/testing/`), known issues, remaining work, and the milestone row in `docs/development/status.md`. Keep reference docs for stable knowledge; put per-run results in `docs/testing/`.

## Phase 7 - Report
End each substantial task with: 1 Objective, 2 Implementation completed, 3 Files changed, 4 Build results, 5 Test results, 6 Acceptance-criteria status, 7 Known issues, 8 Remaining work, 9 Recommended next step. Label each item implemented / built / tested / verified / blocked.

## Engineering rules

**Mecanum, not differential drive.** Four mecanum wheels. Always verify: wheel arrangement (roller pattern, "X" vs "O"), roller orientation per wheel, joint axes, wheel order and sign conventions, forward/inverse kinematics, controller command/state interfaces, odometry math, lateral (vy) and rotational (wz) motion, and Gazebo plugin compatibility. A strafe test and a rotate test are mandatory acceptance items for any drive change.

**Robot description.** Modular Xacro (materials, constants, inertia macros, chassis, wheels, fork, sensors, gazebo extensions, ros2_control, main). `package://` mesh paths only; no absolute paths. Verify mass, inertia, collision, joint origins, TF tree.

**Simulation.** Verify Gazebo Fortress support for spawning, physics, sensors, controllers, ros2_control, ROS-Gazebo bridging and multiple instances. URDF->SDF conversion does not preserve everything: check plugins, `preserveFixedJoint`, friction tags, sensor tags after conversion.

**Navigation.** Verify each Nav2 plugin exists in the installed Humble Nav2 before using it. Get a working baseline (e.g. DWB or RPP, or MPPI with the omni model) before alternatives. Mecanum robots need a holonomic-aware controller config (`vy` allowed); do not copy diff-drive limits.

**Localization / TF.** Define one owner per transform. Default: `map->odom` = localization (AMCL/SLAM); `odom->base_footprint` = exactly one of controller odometry or EKF (never both); `base_link->sensors` = robot_state_publisher. Finalize in `06_odometry_and_tf.md` after implementation.

**Forklift and docking.** Explicit lift states, limits, height feedback if available, fault handling. Docking has measurable alignment tolerances, timeouts, and recovery. Simulation results do not establish physical safety - say so.

**Task management.** Explicit state machine; ROS 2 actions for long-running operations; handle cancel, timeout, navigation/docking/lift failure, and safe stop.

**Web integration.** Planned: REST API HTTP 8000; rosbridge WebSocket 9090; separate video server (reference repo uses `web_video_server` on 8080 - verify). Validate every web command (schema, ranges, state); web input must never bypass safety limits or the command arbitration path.

**Safety and change control.** No destructive commands without explicit approval. Ask before installing system dependencies or changing system configuration. Ask before major architecture changes. Preserve existing work (the workspace already contains `mecanum_forklift_description`, `_gazebo`, `_teleop` plus colcon `build/ install/ log/`). Never conceal failures or invent results. State assumptions and unverified behaviour explicitly.

## Verification sources (in order of preference)
1. The local machine: `ros2 pkg`, `dpkg -l`, `apt-cache policy`/`depends`, installed headers and `share/` metadata, `ign gazebo --versions`.
2. Official docs **for the Humble branch**: docs.ros.org/en/humble, docs.nav2.org (check the distro/version selector), gazebosim.org/docs/fortress, control.ros.org (Humble), and the package's `humble` branch source on GitHub.
3. Reference repo source (for design ideas only).
Anything not verified from 1-2 goes into "Unverified / open questions" in the relevant reference doc.

## Example invocations
`/amr-forklift-mecanum inspect the reference repository` - `... review the architecture` - `... design the robot description` - `... debug the mecanum controller` - `... configure SLAM Toolbox` - `... review Nav2 parameters` - `... test pallet docking` - `... investigate this colcon build failure`.
