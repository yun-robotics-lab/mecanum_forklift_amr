# CLAUDE.md - AMR Forklift Mecanum Wheel

## Purpose
Autonomous mobile robot (AMR) forklift with four mecanum wheels and a fork lift: mapping, localization, navigation, pallet docking, missions, web dashboard. Developed **simulation-first**; hardware comes last.

## Target environment (fixed)
Ubuntu 22.04 LTS - ROS 2 **Humble** - Gazebo **Fortress** (use the `ign` CLI and `ignition-gazebo-*` plugin names) - URDF/Xacro - Python + C++ - one colcon workspace at this directory, packages in `src/`.
Never switch to Ubuntu 24.04, Jazzy or Gazebo Harmonic. Verify any package/API/plugin for Humble + Fortress before recommending it.

## Reference repository
https://github.com/boyloy21/autonomous-mini-forklift - architectural reference only. It is **differential drive**; do not reuse its drive/odometry/MPPI-model/ros2_control config as is. Read it from a clone outside this project; never modify it.

## Architecture (summary - details in the skill references)
Packages named `mecanum_forklift_<role>`. Existing: `description`, `gazebo`, `teleop` (baseline, unverified). Planned: `interfaces`, `control`, `localization`, `navigation`, `lift`, `vision`, `docking`, `task_manager`, `web_bridge`, `bringup`. Authoritative list: `.claude/skills/amr-forklift-mecanum/references/01_project_architecture.md`.
Key rules: mecanum (holonomic) not diff-drive; one owner per TF transform; web/mission commands go through arbitration and safety limits, never straight to `cmd_vel`/lift; planned ports REST 8000, rosbridge 9090, video server separate.

## Skill
For any robotics task on this project (description, Gazebo, control, odometry/TF, SLAM, Nav2, interfaces, lift, vision/docking, missions, web, bringup, build/test/debug), use the **`amr-forklift-mecanum`** skill (`/amr-forklift-mecanum <task>`). Roadmap: `docs/development/roadmap.md`; status: `docs/development/status.md`; decisions: `docs/decisions/`.

## Workflow
Understand -> Inspect -> Plan -> (approval) -> Implement -> Build/Test -> Document -> Report. Inspect files and installed packages before assuming anything exists.

## Conventions
- ROS 2 Humble style: `ament_cmake` for C++/interfaces/description, `ament_python` for Python nodes; PEP 8, ROS 2 C++ style; params in YAML, not code; `use_sim_time` as a launch arg; `package://` mesh paths, no absolute paths.
- Topics: lowercase snake_case; velocity chain `cmd_vel_teleop|nav|dock -> arbitration -> cmd_vel` (once arbitration exists).
- Docs: English, concise, no duplication; update `docs/development/status.md` when a milestone changes state.

## Testing expectations
Build affected packages; run `colcon test` where tests exist; validate Xacro/YAML; run simulation checks for behaviour changes; strafe + rotate tests for any drive change. Use the status words **implemented / built / tested / verified / blocked** precisely. A feature is not "working" because files exist or the build passes. Never hide failures or invent results; mark untestable items **unverified** with the reason.

## Approval policy
Ask before: starting a new milestone, creating packages, changing agreed architecture, adding dependencies. Wait for approval of the plan for substantial work. Final reports follow the 9-point format in the skill.

## Safety and change control
- No destructive commands (delete/overwrite/reset/force) without explicit approval; inspect targets first.
- Ask before installing system packages or changing system configuration (`apt`, udev, env files).
- Preserve existing work and unrelated changes; `build/ install/ log/` are colcon outputs - do not delete without asking.
- The directory is currently **not a Git repository**; do not run `git init` without asking.
- Simulation does not validate physical safety; say so wherever safety is discussed.
