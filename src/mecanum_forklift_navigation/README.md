# mecanum_forklift_navigation

Nav2 (Humble, Nav2 1.1.20) configuration and launch files for the mecanum forklift: NavFn planner, **MPPI Omni local controller (default, ADR-0010)**, DWB as fallback, curated navigation map. **Status: tested in simulation 2026-11-23, not verified** (`docs/development/status.md`, report `docs/testing/2026-11-23_m9_navigation.md`): 20/20 goals reached, 17/20 within 5 cm / 5 deg, no contacts; Nav2 does not give up by itself on a goal blocked after planning (callers must time out and cancel).

## Files
| File | What |
|---|---|
| `launch/navigation.launch.py` | planner, controller, smoother, behaviors, BT navigator, waypoint follower, velocity smoother; args `controller_type` (default `mppi`), `params_file`, `map`, `namespace`, `use_sim_time`, ... |
| `launch/single_bringup.launch.py` | localization (AMCL) + navigation + RViz; args `map`, `nav_map`, `rviz`, `set_initial_pose`, `initial_x/y/yaw` |
| `launch/rviz.launch.py`, `rviz/nav2_default_view.rviz` | RViz |
| `config/nav2_params.yaml`, `controller_dwb.yaml`, `controller_mppi.yaml`, `controller_rotation_shim.yaml` | parameters (footprint includes the fork tines) |
| `maps/simple_warehouse_nav.{yaml,pgm}` | SLAM map plus floor-level objects the lidar cannot see |
| `scripts/nav_trials.py`, `scripts/nav_failures.py` | development tools (not installed): 10-goal chain with ground-truth error / clearance / strafing / pushed objects, and the failure-injection cases |
| `scripts/make_nav_map.py` | builds that map: `make_nav_map.py <slam_map.yaml> <world.sdf> <output_basename>` |

Velocity chain: `controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_nav_safe -> twist_mux -> cmd_vel_mux -> cmd_vel_gate -> cmd_vel` (teleop wins in the mux).

## Run (to start milestone 9 testing)
```bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false
ros2 launch mecanum_forklift_navigation single_bringup.launch.py
# then set a goal in RViz (Nav2 Goal) or: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...
```

## Test
```bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false
ros2 launch mecanum_forklift_navigation single_bringup.launch.py rviz:=false
python3 src/mecanum_forklift_navigation/scripts/nav_trials.py --label mppi --csv /tmp/run.csv    # 10 goals, ~5 min
python3 src/mecanum_forklift_navigation/scripts/nav_failures.py                                   # failure cases, ~15 min
```
Start from the spawn pose with AMCL localised. Results and findings: `docs/testing/2026-11-23_m9_navigation.md`. Verify every Nav2 plugin name exists in the installed Humble Nav2 before using it.

## Developing
- Mecanum robot: allow `vy`; do not copy differential-drive limits.
- AMCL uses the unmodified SLAM map; only the costmaps use the curated map.
- Package has no `setup.py`/tests; it is an `ament_cmake` data package.
- Footprint is the unloaded robot incl. tines; a carried pallet adds ~0.07 m at the front (not yet handled).
- Goal tolerances are 4-5 cm / 0.07-0.08 rad: tighter values make DWB dither and are below AMCL's own accuracy (3-7 cm).

## Update parameters
- Planner, costmaps, behaviors, BT: `config/nav2_params.yaml`; controller: `config/controller_mppi.yaml` (default), `controller_dwb.yaml`, `controller_rotation_shim.yaml`. Select with `controller_type:=mppi|dwb|rotation_shim|mppi_light`.
- `controller_mppi_light.yaml` is a lighter MPPI (batch 1000, 40 steps, 10 Hz) for multi-robot runs on a CPU-only machine; keep it in sync with `controller_mppi.yaml`.
- Footprint, goal tolerances and speed limits are in these files. Rebuild with `colcon build --packages-select mecanum_forklift_navigation` (use `--symlink-install` to skip rebuilds when editing YAML), then relaunch. Live changes: `ros2 param set /controller_server <name> <value>`.

## Multi-robot / namespace
`single_bringup.launch.py namespace:=robot1 use_namespace:=true` runs the stack under `/robot1`. Costmaps use absolute topics (`/scan`, `/nav_map`) that the launch file rewrites with the namespace; `/tf` is remapped to `tf`. Recovery behaviors publish on `cmd_vel_nav`, so they pass the smoother, mux, e-stop and gate. For dev scripts set `ROBOT_NS=/robot1`.
