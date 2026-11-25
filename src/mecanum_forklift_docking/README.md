# mecanum_forklift_docking

`DockPallet` action: pick up or drop a pallet using `opennav_docking`, ArUco markers and our own holonomic final alignment. Simulation-tested only. Decisions: ADR-0010, ADR-0011. Test reports: `docs/testing/2026-11-15_m11b_docking.md`, `2026-11-19_m11b_*.md`, `2026-11-24_m11c_pick_carry.md`.

## Nodes and files
| File | What |
|---|---|
| `mecanum_forklift_docking/dock_core.py`, `align_core.py` | ROS-free logic: dock database, holonomic final alignment (10 mm / 10 mm / 2 deg) |
| `mecanum_forklift_docking/workflow_node.py` | `pallet_dock_workflow`: the `dock_pallet` action server |
| `config/docking_server.yaml` | `opennav_docking` server (SimpleChargingDock plugin, vx/wz controller) |
| `config/workflow.yaml` | Workflow parameters (below) |
| `config/dock_database.yaml` | Docks: `pallet_pick_1`-`3`, `pallet_drop_1`-`4`; only `pick_1` and `drop_1` are tested (+ test-only `test_lift_out_of_range`) |
| `config/dock_database_warehouse_v2.yaml` | `warehouse_v2` docks: `pallet_pick_1` (floor), `pallet_drop_1`-`3`, `rack_pick_1`, `rack_drop_1`-`2`; not formally tested (a floor mission and a rack-to-rack transfer appear once each in `docs/videos/`). Picked automatically by `world:=warehouse_v2.sdf` in bringup (standalone: `docking.launch.py dock_database:=... nav_map_default:=.../warehouse_v2_nav.yaml`) |
| `launch/docking.launch.py` | Starts `docking_server`, its lifecycle manager and `pallet_dock_workflow` |
| `scripts/` | Dev tools: `dev_stack.sh`, `dock_trials.py`, `pick_trials.py`, `*_failures.py`, `engagement_check.py`, `drop_check.py` |

## Interfaces
| Name | Type | Direction |
|---|---|---|
| `dock_pallet` | action `mecanum_forklift_interfaces/DockPallet` (`dock_id`, `navigate_to_staging`, `undock_after`) | server |
| `aruco_dock_pose` | `PoseStamped` | subscribes (vision) |
| `cmd_vel_dock` | `Twist` | publishes (twist_mux priority 50, then the gate limits) |
| `set_lift_height`, `navigate_to_pose`, `dock_robot`/`undock_robot` | actions | clients (lift, Nav2, opennav) |
| `nav_map_server/load_map` | `nav2_msgs/LoadMap` | client (map swap after a pick or drop) |

Sequence: marker -> staging navigation (unloaded footprint) -> opennav approach -> holonomic alignment -> lift (or lower) -> optional back-out. Error codes are listed in `mecanum_forklift_interfaces/action/DockPallet.action`.

## Run
Needs simulation, navigation, lift and vision running:
```bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false
ros2 launch mecanum_forklift_navigation single_bringup.launch.py
ros2 launch mecanum_forklift_vision vision.launch.py
ros2 launch mecanum_forklift_docking docking.launch.py
ros2 action send_goal /dock_pallet mecanum_forklift_interfaces/action/DockPallet \
  "{dock_id: pallet_pick_1, navigate_to_staging: true, undock_after: true}" --feedback
```
Dev shortcut: `scripts/dev_stack.sh start|stop` (run `stop` alone; logs in `~/.cache/mecanum_logs`). The `mecanum_forklift_bringup` package starts all of this per robot.

## Update parameters
- **Rack docks** (`warehouse_v2`): an entry with `approach_lift` raises the fork at the staging pose, before driving up to the rack. A pick then lifts to `lift`, reverses straight by `backoff` (no `undock_robot`) and lowers to `carry_lift`; a drop lowers to `lift` in the rack, reverses by `backoff` and sets `fork_after`. A rack drop fixes its `map` target in `odom` before raising, because the raised load enters the lidar plane. The fork is lowered only after a completed back-out; with `undock_after: false` it stays at rack height. A floor pick after a failed rack operation lowers the fork first. The rack pallet stands 3 cm over the deck's front edge so the chassis stays 6 cm clear. The drop has no marker: its depth accuracy is AMCL's (4-7 cm measured on floor drops; tolerated about 5 cm short, 5.8 cm deep). `pallet:` on a pick and a drop entry makes the drop fail with `WRONG_PALLET` for another pallet (floor-drop maps assume `pallet_1`).
- **Docks** (`dock_database.yaml`): each entry has `type`, `frame`, `pose` `[x, y, yaw]`, `marker_id`, `lift` (m, limit 0.12), `operation: drop` for drops, `staging_pose`/`final_pose` (drops), `map_after_pick`/`map_after_drop` (curated maps in the navigation package). Add a dock by adding an entry; the web API lists every dock without `test_only`.
- **Workflow** (`workflow.yaml`): `stop_distance` (0.245 m, tied to fork geometry: do not reduce), `total_timeout`, `align_timeout`, `tol_long/lat/yaw_deg`, `latch_distance`, `unloaded_footprint`/`loaded_footprint`, `staging_x_offset` (must equal the plugin value in `docking_server.yaml`).
- Rebuild the package after editing (config is installed), then relaunch docking.

## Test
```bash
colcon build --packages-select mecanum_forklift_interfaces mecanum_forklift_docking
colcon test --packages-select mecanum_forklift_docking && colcon test-result --verbose
```
Unit tests cover `align_core` and `dock_core` (+ flake8/pep257). Simulation checks use the scripts with the stack running: `python3 scripts/pick_trials.py 5`, `dock_trials.py`, `*_failures.py` (failure injection). For a robot in a namespace set `ROBOT_NS=/robot1`. Simulation does not validate physical safety; marker size, lift height and tolerances are sim placeholders.
