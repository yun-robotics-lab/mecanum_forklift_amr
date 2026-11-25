# 2026-11-06 - Milestone 7, SLAM and mapping

Setup: `mecanum_forklift_gazebo.launch.py gui:=false` (simple_warehouse.sdf), `mecanum_forklift_localization/launch/mapping.launch.py` (slam_toolbox 2.6.10 online async, `config/slam_toolbox_mapping.yaml`), RTF ~0.5. The tour was a script (scratch dir, not in the repo): waypoints in the odom frame at 0.2 m/s with heading held at 0 (so strafing and backward driving are included), 360 deg rotations in place both ways, and a second lap. Lanes at y = +-0.45 m avoid the pallet, cones and shelf posts. Map saved with `map_saver_cli` to `src/mecanum_forklift_localization/maps/simple_warehouse.{yaml,pgm}` (121 x 102 cells, 0.05 m, origin -3.04, -2.54).

| Check | Result | Status |
|---|---|---|
| Scan sanity (before SLAM) | frame `lidar_link`, 360 beams all finite, no occlusion by the robot, wall ranges 2.99 / 2.50 / 3.00 / 2.49 m at 0 / 90 / 180 / -90 deg vs true 3.0 / 2.5 / 3.0 / 2.5 | tested |
| SLAM pose vs ground truth (`map->base_footprint` vs Gazebo pose) | end of tour 6.5 mm / 0.08 deg; along the run max 12.1 mm, mean 5.8 mm, max yaw 0.84 deg; criterion <= 5 cm / 2 deg | tested, met |
| Map dimensions | interior 5.975 x 5.000 m by wall-cell centroids (true 6.000 x 5.000: -2.5 cm, 0.0 cm; each wall within 1.5 cm of its true face); criterion within 5 cm | tested, met (see caveat) |
| Wall thickness in map | 2 cells (10 cm) on three walls, 1 cell on the east wall; criterion <= 2 cells | tested, met |
| Save and reload | `map_server` (via lifecycle manager) reaches `active`, publishes a 121 x 102 grid, 0.05 m, same origin; 727 occupied cells equal the PGM | tested |
| `map->odom` owner / TF | `/tf` publishers: `robot_state_publisher`, `slam_toolbox`, `wheel_odometry` (one per frame set); 0 TF warnings (extrapolation, dropping, transform failures) in sim and SLAM logs | tested |

## Caveats and findings
- **Dimension estimator.** The pixel grid is 5 cm, so a wall lands in 1-2 cells depending on where it falls relative to the cell boundary. I tried three estimators on the same map: outermost wall cell 6.000 x 5.050 m, innermost cell edge 5.900 x 4.900 m (-10 cm, would fail), cell centroid 5.975 x 5.000 m (reported above). The centroid is the unbiased one for a wall that straddles 1-2 cells, but I chose it after seeing the others; treat the real accuracy as +-1 cell (5 cm) at this resolution.
- **Pallets and boxes are not in the map.** The lidar plane is at z = 0.224 m; the pallets (0.066 m) and boxes (< 0.2 m) are below it, so only walls and shelf posts are mapped. This matters for Nav2 costmaps (M9) and pallet detection/docking (M11), which cannot rely on this lidar for pallet-level obstacles.
- **Loop closure is unverified.** Wheel odometry has almost no drift in the sim, so SLAM had nothing to correct; the low pose error mostly reflects the odometry. The optional drift-injection test was not requested.
- **Unknown cells load as free.** `map_saver_cli` trinary mode writes unknown as 205 with `free_thresh 0.25`, so the 54 unknown cells load as free (0). Harmless here (they are outside the walls) but note it before AMCL/Nav2 (M8/M9).
- The tour is scripted; mapping by hand with the PS4 was not tested. Hardware lidar noise, glossy surfaces and real slip are unverified. Simulation does not validate physical safety.

## Runbook
```
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false     # or gui:=true
ros2 launch mecanum_forklift_localization mapping.launch.py            # opens RViz (config/mapping.rviz); rviz:=false to skip
# drive with ps4_teleop.launch.py or keyboard_node at <= 0.2-0.3 m/s; include a loop
ros2 run nav2_map_server map_saver_cli -f src/mecanum_forklift_localization/maps/<name>
```

## Addendum: RViz in the mapping launch (2026-11-06)
`mapping.launch.py` now also starts `rviz2` with `config/mapping.rviz` (fixed frame `map`; displays: Map `/map` transient-local, LaserScan `/scan` best-effort, RobotModel, TF; Odometry off). Launch argument `rviz` (default true). Tested: RViz node starts and subscribes to `/map`, `/map_updates`, `/scan`, `/robot_description` with no errors in the launch log. The rendered picture was not inspected by me (no screenshot), so what RViz draws is unverified; `rviz2` added as `exec_depend` (already installed).
