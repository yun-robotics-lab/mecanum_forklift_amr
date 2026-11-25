# mecanum_forklift_localization

Mapping (SLAM Toolbox) and localization (AMCL with the omni motion model), plus saved maps. Decisions: ADR-0005 (mapping), ADR-0006 (AMCL). Reports: `docs/testing/2026-11-06_m7_slam_mapping.md`, `2026-11-06_m8_amcl_localization.md`.

## Files
| File | What |
|---|---|
| `launch/mapping.launch.py` | slam_toolbox online async + RViz (args `use_sim_time`, `rviz`) |
| `launch/localization.launch.py` | map_server + AMCL + lifecycle manager (args `map`, `params_file`, `rviz`, `set_initial_pose`, `initial_x/y/yaw`, `namespace`, `autostart`, ...) |
| `config/slam_toolbox_mapping.yaml`, `config/amcl.yaml` | parameters |
| `config/mapping.rviz`, `config/localization.rviz` | RViz views |
| `maps/simple_warehouse.{yaml,pgm}` | saved map of the sim warehouse (default). `my_map.*` is an extra map whose origin is not documented |

TF: `map -> odom` is owned by SLAM Toolbox while mapping or by AMCL while localizing - never run both.

## Run (start the simulation first)
```bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false
ros2 launch mecanum_forklift_localization mapping.launch.py                 # drive the robot with teleop
ros2 run nav2_map_server map_saver_cli -f src/mecanum_forklift_localization/maps/<name>
# localization on a saved map; initial pose must be roughly right (<= ~0.5 m, 30 deg)
ros2 launch mecanum_forklift_localization localization.launch.py initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```
Rebuild after saving a map so it is installed (`colcon build --packages-select mecanum_forklift_localization`).

## Test
No automated tests. Procedure and results (pose error versus ground truth `/odom_ground_truth`, map size versus the 6.0 x 5.0 m world, wrong-initial-pose recovery) are in the two reports above. Status: verified for simulation only; loop closure and hardware unverified; AMCL values are tuned on a noiseless sim.

## Developing
- Keep `scan` range 0.12-8.0 m consistent with the lidar sensor.
- A gross initial-pose error (1 m / 90 deg) does not recover; use global localization.

## Update parameters
- AMCL: `config/amcl.yaml` (particles 1000-5000, update thresholds, laser model). `config/amcl_light.yaml` is a lighter copy (300-1000 particles) for multi-robot runs; keep both in sync. Pass it with `amcl_params_file:=<path>` where the launch supports it.
- SLAM Toolbox: `config/slam_toolbox_mapping.yaml`.
- After editing: `colcon build --packages-select mecanum_forklift_localization`, relaunch. Live: `ros2 param set /amcl <name> <value>`.
- Under a namespace, AMCL and `map_server` run in `/robotN` with `/tf` remapped to `tf`.
