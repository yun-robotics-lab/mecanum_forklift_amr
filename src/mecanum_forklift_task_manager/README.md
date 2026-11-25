# mecanum_forklift_task_manager

Mission manager for the mecanum forklift: runs `PALLET_TRANSPORT` missions (pick, drop, optional return) over Nav2 and the `DockPallet` action with timeouts, bounded retries, cancel and status. **Status: tested in simulation 2026-11-24, not verified** (`docs/testing/2026-11-24_m12_mission.md`).

| Part | What |
|---|---|
| `mission_core.py` | ROS-free state machine (`MissionRunner`, `Policy`); unit tested with a scripted executor |
| `mission_node.py` | `ExecuteMission` action server (`execute_mission`), `MissionStatus` topic (`mission_status`), child clients for `dock_pallet` and `navigate_to_pose` |
| `config/mission.yaml`, `launch/mission.launch.py` | timeouts (sim time), retries; the launch file passes the dock database of the docking package |
| `scripts/` | `mission_trials.py`, `mission_failures.py`, `mission_common.py` (development tools, not installed) |

Error codes of the result: `INVALID_MISSION`, `MISSION_TIMEOUT`, `CANCELED`, `NAV_FAILED`, `BUSY`, `INTERNAL_ERROR`, or the `DockPallet` code of the failed step (`STAGING_FAILED`, `MARKER_NOT_FOUND`, `LIFT_FAILED`, `ALIGN_FAILED`, `TIMEOUT`, ...); `failed_step` is VALIDATE, PICK, DROP or RETURN.
Retried once: `STAGING_FAILED`, `MARKER_NOT_FOUND`, `NAV_FAILED`. Timeouts can be changed with `ros2 param set /mission_manager dock_timeout 60.0`.

```bash
src/mecanum_forklift_docking/scripts/dev_stack.sh start          # sim, Nav2, vision, docking
src/mecanum_forklift_docking/scripts/dev_stack.sh start_mission
ros2 action send_goal /execute_mission mecanum_forklift_interfaces/action/ExecuteMission \
  "{mission_id: m1, mission_type: PALLET_TRANSPORT, pick_dock_id: pallet_pick_1, drop_dock_id: pallet_drop_1, return_pose: {header: {frame_id: map}, pose: {orientation: {w: 1.0}}}}"
python3 src/mecanum_forklift_task_manager/scripts/mission_trials.py 5      # needs the stack
python3 src/mecanum_forklift_task_manager/scripts/mission_failures.py     # failure matrix
```
Limits: one mission at a time (a second is aborted with BUSY), no queue, no persistence, cancelling leaves the fork where it is, the manager never publishes velocity itself.

## Update parameters
`config/mission.yaml` (node key `/**/mission_manager`): `nav_timeout`, `dock_timeout`, `mission_timeout`, retry counts (sim time). Rebuild the package and relaunch (`ros2 launch mecanum_forklift_task_manager mission.launch.py`), or change live with `ros2 param set /mission_manager <name> <value>`.

## Test
```bash
colcon build --packages-select mecanum_forklift_interfaces mecanum_forklift_task_manager
colcon test --packages-select mecanum_forklift_task_manager && colcon test-result --verbose
```
Unit tests cover `mission_core` with a scripted executor (+ flake8/pep257). Simulation trials need the full stack; for a namespaced robot set `ROBOT_NS=/robot1`.
