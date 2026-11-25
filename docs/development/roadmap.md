# Roadmap

Gate rule: a milestone starts only when its dependencies are *verified* (or the user explicitly overrides - record it). Approval is required to start each milestone (see CLAUDE.md). Tolerances marked TBD must be proposed and approved before testing. Milestones 3-5 start with an **audit of the existing packages**.

| # | Milestone | Depends on |
|---|---|---|
| 1 | Requirements & reference analysis | - |
| 2 | Architecture & package dependencies | 1 |
| 3 | Robot description | 2 |
| 4 | Gazebo Fortress simulation | 3 |
| 5 | Mecanum control & teleop | 4 |
| 6 | Odometry & TF | 5 |
| 7 | SLAM & mapping | 6 |
| 8 | Localization | 7 |
| 9 | Autonomous navigation | 8 |
| 10 | Forklift control | 4 (3 for model) |
| 11 | Pallet vision & docking | 9, 10 |
| 12 | Mission management | 9, 10, 11 |
| 13 | Web integration | 12 |
| 14 | Bringup & multi-robot | 9 (full: 13) |
| 15 | Integration & regression | 1-14 |
| 16 | Hardware integration | 15 |

Detail references: subsystem files `.claude/skills/amr-forklift-mecanum/references/NN_*.md`.

## 1. Requirements and reference repository analysis
- **Objective**: written requirements; understanding of what from the reference repo is reusable.
- **Deliverables**: `docs/requirements/requirements.md` completed; reference analysis note (package-by-package reuse verdict); open-question list answered.
- **Tests / acceptance**: user signs off requirements; every reference package has a verdict (reuse / adapt / skip) with reason.
- **Risks**: reference is diff-drive; some code needs unavailable packages. **Approval**: reuse verdicts.

## 2. System architecture and package dependency design
- **Objective**: approved package list, dependency graph, interface list, TF/ownership plan, naming.
- **Deliverables**: updated `01_project_architecture.md`, `docs/architecture/*`, ADRs (0002 drive stack, velocity chain, namespacing).
- **Acceptance**: no dependency cycles; every requirement mapped to a package; user approval.
- **Risks**: over-splitting packages. **Approval**: package creation, ADR-0002.

## 3. Robot description
- **Objective**: validated modular Xacro with correct inertia/collision/TF.
- **Dependencies**: 2. **Deliverables**: cleaned Xacro (resolve the empty placeholder files), frame/dimension table, IMU frame, camera optical frame.
- **Tests**: `xacro` + `check_urdf` clean; RViz TF tree; no absolute paths; mass/inertia review.
- **Acceptance**: all checks pass; TF tree matches documented table; model dimensions confirmed against intended robot.
- **Risks**: mesh-based collision breaking friction; real dimensions unknown. **Approval**: model scale.

## 4. Gazebo Fortress simulation
- **Objective**: reproducible sim with world(s), spawn, sensors, bridge.
- **Deliverables**: world(s) with required system plugins, bridge incl. IMU, sensors verified, headless mode.
- **Tests**: spawn/stability at rest, topic rates (`/scan`, camera, `/clock`), RTF.
- **Acceptance**: robot static for 60 s (drift <= TBD); all declared topics publish at expected rates.
- **Risks**: GPU/render engine, URDF->SDF losses. **Approval**: any apt install (e.g. `ign_ros2_control`).

## 5. Mecanum control and teleoperation
- **Objective**: correct holonomic control via the approved stack; safe teleop.
- **Deliverables**: controller config/nodes, arbitration (`twist_mux`), updated teleop with deadman/timeout, kinematics doc.
- **Tests**: forward / strafe / rotate / combined / timeout (04, 05).
- **Acceptance**: displacement and yaw errors within approved tolerances; stops within TBD s on release or timeout.
- **Risks**: roller orientation/sign errors; ros2_control version skew. **Approval**: ADR-0002, tolerances.

## 6. Odometry and TF
- **Objective**: single-owner TF tree, validated odometry (EKF only if justified).
- **Tests**: `view_frames`, duplicate-publisher check, odom vs ground truth.
- **Acceptance**: one parent per frame; odom error <= TBD over the 04 motion set. **Risks**: dual publishers, covariance tuning.

## 7. SLAM and mapping
- **Objective**: reliable maps of the sim warehouse. **Tests**: map vs known dimensions, loop closure, save/reload.
- **Acceptance**: dimension error <= TBD cm; map file loads in map server. **Risks**: yaw drift from strafing.

## 8. Localization
- **Objective**: AMCL with the omni motion model. **Tests**: pose error vs ground truth; recovery from wrong initial pose.
- **Acceptance**: RMS error <= TBD after a loop. **Risks**: wrong motion model, map/scan mismatch.

## 9. Autonomous navigation
- **Objective**: Nav2 baseline then MPPI-Omni, then route graph (9b). **Tests**: goal set (N TBD), obstacles, cancel, recovery.
- **Acceptance**: success rate and final error per 08; no collisions; strafe used. **Risks**: footprint with fork, controller param names, `nav2_route` API. **Approval**: controller choice, route format.

## 10. Forklift control
- **Objective**: lift state machine, action API, hardware abstraction. **Tests**: 10 §5.
- **Acceptance**: height error <= TBD, all fault paths pass. **Risks**: real actuator unknown; fork under load.

## 11. Pallet vision and docking
- **Objective**: marker/pallet pose and docking. **Dependencies**: 9, 10.
- **Tests**: offset trials, failure injection (11 §5). **Acceptance**: success >= TBD % within tolerance. **Risks**: camera on moving carriage, `opennav_docking` availability/API. **Approval**: docking tolerances, library choice, any ML install.

## 12. Mission management
- **Objective**: task/mission actions with cancel/timeout/recovery. **Tests**: failure-injection matrix (12 §5).
- **Acceptance**: each failure ends in specified state, robot stopped, no orphan goals. **Risks**: concurrency with teleop.

## 13. Web integration
- **Objective**: REST 8000 + rosbridge 9090 + video, validated commands. **Tests**: 13 §6.
- **Acceptance**: forbidden commands rejected; status rate met. **Risks**: auth, port clashes. **Approval**: auth scheme, video technology.

## 14. Bringup and multi-robot
- **Objective**: layered launches and 2-robot simulation. **Tests**: 14 §4. **Acceptance**: reproducible 1- and 2-robot runs with no cross-talk. **Risks**: TF/namespace handling, compute load. **Approval**: TF isolation approach (ADR).

## 15. System integration and regression testing
- **Objective**: scripted end-to-end scenarios and regression suite. **Acceptance**: full mission passes N consecutive runs; regression report archived. **Risks**: flaky timing.

## 16. Future hardware integration
- **Objective**: swap sim for hardware interfaces. **Deliverables**: hardware abstraction implementations, safety review, commissioning plan. **Acceptance**: TBD with hardware owner. **Risks**: physical safety (e-stop, protective fields) - not validated by simulation. **Approval**: all hardware-related steps.
