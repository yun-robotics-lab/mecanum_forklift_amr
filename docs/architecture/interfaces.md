# Interfaces

Package: `mecanum_forklift_interfaces` (ament_cmake, leaf dependency; depends on `std_msgs` and `geometry_msgs`). Rules: `.claude/skills/amr-forklift-mecanum/references/09_custom_interfaces.md`.

| Type | Kind | Owner | Producers | Consumers | Since |
|---|---|---|---|---|---|
| `msg/LiftState` | topic | interfaces | `mecanum_forklift_lift/lift_controller` | task manager, web bridge, docking (not yet written) | v0.0.0 |
| `action/DockPallet` | action | interfaces | tests / `mission_manager` (client) | `mecanum_forklift_docking/pallet_dock_workflow` (server) | v0.1.0 |
| `action/ExecuteMission` | action | interfaces | tests / web layer later (client) | `mecanum_forklift_task_manager/mission_manager` (server) | v0.2.0 |
| `msg/MissionStatus` | topic | interfaces | `mission_manager` (2 Hz + on change) | dashboard / web bridge (later) | v0.2.0 |
| `action/SetLiftHeight` | action | interfaces | task manager, docking (not yet written); a script client in tests | `lift_controller` (server) | v0.0.0 |

 Nav2 needs no custom types (`nav2_msgs/NavigateToPose` is used).

## Code tables
- `LiftState.state`: 0 UNINITIALIZED, 1 HOMING, 2 IDLE, 3 MOVING, 4 FAULT (matches the state machine in reference 10).
- `LiftState.fault_code`: 0 NONE, 1 TIMEOUT, 2 OVER_LIMIT, 3 NO_FEEDBACK, 4 STALL, 5 SAFETY_STOP.
- `SetLiftHeight` result `error_code`: 0 SUCCESS, 1 REJECTED_NOT_IDLE, 2 REJECTED_OUT_OF_RANGE, 3 TIMEOUT, 4 NO_FEEDBACK, 5 STALL, 6 CANCELED, 7 SAFETY_STOP.

Height range and timeouts are controller parameters (sim fork travel 0.12 m), not part of the contract. Feedback and `lift_state` are published at 10 Hz of node (sim) time. Rejected goals are accepted then aborted with a reason code; a measured height outside the limits reuses `RESULT_REJECTED_OUT_OF_RANGE`.

- `DockPallet` result `error_code`: "" success, DOCK_NOT_IN_DB, MARKER_SET_FAILED, STAGING_FAILED, MARKER_NOT_FOUND, CONTROL_FAILED, DOCK_FAILED, ALIGN_FAILED, LIFT_FAILED, MAP_UPDATE_FAILED, NOT_CARRYING, UNDOCK_FAILED, TIMEOUT, CANCELED, SERVER_UNAVAILABLE; feedback `state`: SET_MARKER, STAGING, DOCKING, ALIGNING, LIFTING | LOWERING, UNDOCKING, DONE. The dock database entry (`operation: pick|drop`) decides the behaviour, not the goal.
- `ExecuteMission` result `error_code`: "" success, INVALID_MISSION, MISSION_TIMEOUT, CANCELED, NAV_FAILED, BUSY, INTERNAL_ERROR or a DockPallet code; `failed_step`: VALIDATE | PICK | DROP | RETURN.

## Changelog
- v0.0.0 (2026-11-14): `LiftState`, `SetLiftHeight` added. Changes to a type: append fields only; rename = new type (type hashes are not versioned in Humble).
- v0.1.0 (2026-11-19): `DockPallet` added (11b/11c: states STAGING, error codes ALIGN_FAILED, MAP_UPDATE_FAILED added later; 2026-11-24: LOWERING, NOT_CARRYING).
- v0.2.0 (2026-11-24): `ExecuteMission`, `MissionStatus` added; the package now depends on `geometry_msgs`.
