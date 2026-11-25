# 15 - Testing and Debugging

## 1. Purpose
How to prove (or disprove) that something works, how to debug systematically, and how to report honestly. Applies to every milestone.

## 2. Status vocabulary
implemented (written) -> built (compiles) -> tested (executed, result recorded) -> verified (acceptance criteria met with evidence); blocked. Never skip levels in reports. Never invent logs or numbers; paste or summarize the real command output.

## 3. Test layers
| Layer | Tools | When |
|---|---|---|
| Build | `colcon build --packages-select <pkg> [--cmake-args ...]`; warnings reviewed | every change |
| Static | `ament_flake8`, `ament_pep257`, `ament_cppcheck`/`cpplint`, `ament_xmllint` via `colcon test` | where `ament_lint_auto` is configured |
| Unit | `pytest` (ament_cmake_pytest / ament_python), `gtest` | kinematics, validators, state machines |
| Config validation | `xacro` + `check_urdf`, YAML load + parameter-name check against node params (`ros2 param dump`), launch file `ros2 launch --show-args` | description/config changes |
| Integration | `launch_testing` or scripted launch + assertions | multi-node |
| Simulation | Gazebo headless (`ign gazebo -s -r`) + scripted scenarios; ground truth from Gazebo | each milestone |
| Regression | scripted scenario suite re-run on each milestone completion; results logged | M15 |
Check what is actually installed first (`which check_urdf`, `pip list`); ask before installing.

## 4. Standard commands
```bash
cd ~/Personal_Project/mecanum_forklift_amr
source /opt/ros/humble/setup.bash
colcon build --packages-select <pkg> 2>&1 | tee /tmp/build.log
source install/setup.bash
colcon test --packages-select <pkg> && colcon test-result --verbose
ros2 node list; ros2 topic list -t; ros2 topic hz <t>; ros2 topic info <t> -v
ros2 service list; ros2 action list
ros2 control list_controllers; ros2 control list_hardware_interfaces   # if ros2_control is installed
ros2 run tf2_tools view_frames; ros2 run tf2_ros tf2_echo <a> <b>
ros2 param get/dump <node>; ros2 lifecycle get <node>
ign topic -l; ign topic -e -t <topic>            # Gazebo Fortress
ss -ltnp | grep -E '8000|9090|8080'              # web ports
```
Stale workspace: `build/`, `install/`, `log/` exist. Do not delete them without approval; prefer `--packages-select` and `colcon build --cmake-clean-cache` only on request.

## 5. Systematic debugging
1. Reproduce with the smallest command; capture exact error and the first error, not the last.
2. Classify (below).
3. Form a hypothesis; find the cheapest observation that falsifies it (topic echo, TF echo, log level, Gazebo topic).
4. Change one variable; re-test; record.
5. Fix the root cause, add a regression check, document in `docs/testing/`.

| Class | Typical evidence |
|---|---|
| Build/dependency | missing `package.xml` dep, CMake `find_package`, stale cache, wrong distro package |
| Environment | wrong `source`, `ROS_DOMAIN_ID`, `IGN_GAZEBO_RESOURCE_PATH`, no display/GPU |
| Model/description | `check_urdf` errors, SDF conversion diff, inertia warnings |
| Simulation | plugin load failures in Gazebo log (`-v 4`), RTF drop |
| Communication | topic type/QoS mismatch (`ros2 topic info -v`), bridge direction |
| TF | duplicate publishers, extrapolation errors, wrong frame ids |
| Controller | not active, interface name mismatch, limits/timeouts |
| Algorithm/tuning | works in some cases only, oscillation, drift |
| Safety/logic | missing timeout/cancel/fault path |
Use `use_sim_time:=true` consistently; use `RCUTILS_LOGGING_BUFFERED_STREAM=1` and `--ros-args --log-level <node>:=debug` to increase logs; `ros2 bag record` for repro.

## 6. Acceptance criteria rules
Criteria must be measurable (number + unit + tolerance + sample size) and written **before** testing. A feature passes only when every criterion is met with recorded evidence; otherwise it is "tested - failing" or "partially verified" with the reason. Simulation evidence is labelled "simulation".

## 7. Test report template (put in `docs/testing/`)
```
Test: <name>   Milestone: <n>   Date: <YYYY-MM-DD>   Commit/state: <hash or "no git">
Setup: <launch cmd, world, params file rev>
Procedure: <steps>
Criteria: <list with tolerances>
Results: <measured values / log excerpts>
Outcome: PASS | FAIL | PARTIAL | UNVERIFIED (<why>)
Issues found / follow-ups:
```

## 8. Common pitfalls specific to this project
- Using Harmonic/Jazzy naming (`gz sim`, `gz-sim-*`, `GZ_SIM_RESOURCE_PATH`).
- Assuming differential-drive behaviour (no vy) in Nav2/AMCL/MPPI configs.
- Two sources for `odom->base_footprint`.
- Testing only forward motion; always test strafe and rotation.
- Declaring success because `colcon build` passed.
