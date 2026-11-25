# mecanum_forklift_interfaces

Custom ROS 2 messages and actions shared between packages. Leaf package (depends only on `std_msgs`); no code, only definitions.
Contract table and changelog: `docs/architecture/interfaces.md`. Design rules: `.claude/skills/amr-forklift-mecanum/references/09_custom_interfaces.md`. Decision: ADR-0007.

## Contents
| Type | File | Purpose |
|---|---|---|
| `LiftState` (topic) | `msg/LiftState.msg` | Lift state, fault code, height, velocity, target, feedback flag |
| `SetLiftHeight` (action) | `action/SetLiftHeight.action` | Move the fork to an absolute height; result has a reason code |

Code tables (state, fault, result codes) are constants inside the files; see them with `ros2 interface show`.

## Build and inspect
```bash
cd ~/Personal_Project/mecanum_forklift_amr
colcon build --packages-select mecanum_forklift_interfaces
source install/setup.bash
ros2 interface show mecanum_forklift_interfaces/msg/LiftState
ros2 interface show mecanum_forklift_interfaces/action/SetLiftHeight
```

## Test
```bash
colcon test --packages-select mecanum_forklift_interfaces && colcon test-result --verbose
```
Only lint tests exist. Real checks happen in the consumers (`mecanum_forklift_lift`).

## Developing
- Add a type: put the file in `msg/` or `action/` (CamelCase), list it in `CMakeLists.txt` under `rosidl_generate_interfaces`, add a row to `docs/architecture/interfaces.md`.
- Changing an existing type breaks bags and mixed builds (Humble has no type versioning): append fields only, rename = new type, then **rebuild all dependents** (clean `build/install` of this package if layouts look stale).
- Units in comments (m, rad, m/s); constants UPPER_CASE; no magic numbers.
- Dock and mission types are planned for milestones 11 and 12, not here yet.

## Namespaces
Interface names are relative in all clients and servers (`/robot1/dock_pallet`, `/robot1/execute_mission`, ...), so nothing here changes for multi-robot use. Adding a field changes every dependent package: rebuild `mecanum_forklift_interfaces` and everything that depends on it.
