# 09 - Custom Interfaces

## 1. Purpose
Rules for messages, services and actions shared between packages. Interfaces are contracts: change them deliberately.

## 2. Package and naming
- One package `mecanum_forklift_interfaces` (ament_cmake, `rosidl_default_generators` build dep, `rosidl_default_runtime` exec dep, `member_of_group rosidl_interface_packages`). **Not yet created.**
- Files: `msg/CamelCase.msg`, `srv/CamelCase.srv`, `action/CamelCase.action`; fields `snake_case`; constants UPPER_CASE; always include `std_msgs/Header` where time/frame matter; units in comments (m, rad, m/s).
- Prefer existing types first: `geometry_msgs`, `nav_msgs`, `nav2_msgs/NavigateToPose`, `sensor_msgs/JointState`, `diagnostic_msgs`, `std_srvs`. Create custom types only when no standard fits.

## 3. Candidate contracts (from the reference repo; reuse after review)
Reference `forklift_interfaces` has 6 actions (`ExecuteMission`, `ExecuteTask`, `SetLiftHeight`, `DockRobot`, `UndockRobot`, `PalletDockWorkflow`), 5 services (`ClearQueue`, `GetTaskStatus`, `ReserveEdge`, `SetLiftHeight`, `StartDocking`) and ~40 messages (mission/task/lift/safety/diagnostics). Example read: `ExecuteTask.action` goal = `task_id`, `task_type` (SINGLE_NAVIGATION, PATROL_POINT, PICK_PALLET_DOCK, DROP_PALLET_DOCK, CHARGING_DOCK, STATION_DOCK), `goal_pose`, `waypoints[]`, `dock_id`, `target_height`, `station_wait_duration`; result `success`, `error_code`, `message`; feedback `current_step`, `step_index`, `total_steps`, `state`, `progress`.
`SetLiftHeight.action` appears in the reference as `target_height` (m), `speed` (m/s).
Do **not** copy 40 messages. Start with the minimum for the next milestone (typically: `SetLiftHeight`, lift state, `ExecuteTask`, dock request/result) and grow.
Notes: reference uses `string task_type` with an unspecified enum; prefer constants or `uint8` states + a string `error_code` taxonomy (document it). Both a service and an action named `SetLiftHeight` exist in the reference - avoid duplicates.

## 4. Design rules
- Long-running (> ~1 s) or cancellable operations -> **action**; query/config -> **service**; streaming state -> **topic**.
- Every action defines: preconditions, cancel semantics, failure codes, timeouts, feedback rate.
- Backward compatibility: ROS 2 type hashes are not versioned in Humble; changing a type breaks bags and mixed-version nodes. Add fields at the end; rename = new type; record in the interface changelog.
- Frame and unit conventions in comments; no magic numbers (use constants).

## 5. Testing
- Build the package; `ros2 interface show <type>` for each.
- Round-trip test in each consumer (publish/subscribe or call action) with `colcon test`.
- Rosbridge/JSON compatibility for anything the web uses (13): avoid exotic nested types; check generated JSON by calling via rosbridge.
Acceptance: builds clean; all consumers compile; contract table in docs matches files.

## 6. Failure modes
Message field changed without rebuilding dependents; missing `<depend>`; action result not set on cancel; Python and C++ generating different layouts after stale build (clean `build/install` for the interface package).

## 7. Documentation
`docs/architecture/interfaces.md` (to be created in M2): table of type, owner package, producers, consumers, version/changelog.
