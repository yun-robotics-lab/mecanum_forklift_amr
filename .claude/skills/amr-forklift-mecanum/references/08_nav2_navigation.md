# 08 - Nav2 Navigation

## 1. Purpose
Configure Nav2 on Humble for a holonomic forklift. Establish a working baseline first; add features one at a time.

## 2. Compatibility (verified 2026-11-02, installed Nav2 1.1.20)
Present: `nav2_bringup`, `nav2_bt_navigator`, `nav2_controller`, `nav2_planner`, `nav2_smoother`, `nav2_behaviors`, `nav2_costmap_2d`, `nav2_waypoint_follower`, `nav2_velocity_smoother`, `nav2_collision_monitor`, `nav2_amcl`, `nav2_map_server`, planners (`navfn`, `smac`, `theta_star`), controllers **DWB** (`nav2_dwb_controller`), **MPPI** (`nav2_mppi_controller`, header shows `OmniMotionModel`), **Regulated Pure Pursuit**, **Rotation Shim**, and **`nav2_route`** (route graph server).
Not installed: `opennav_docking` (apt candidate 0.0.2; Humble Nav2 does not ship `nav2_docking`).
Do not read Jazzy-or-newer Nav2 docs for parameter names; use the Humble/1.1.x docs and the installed default params.

## 3. Architecture
`bt_navigator` (NavigateToPose / NavigateThroughPoses) -> `planner_server` (global) -> `smoother_server` -> `controller_server` (local, + local costmap) -> `cmd_vel_nav` -> velocity smoother -> collision monitor -> arbitration -> drive. `behavior_server` for recoveries; `lifecycle_manager` per group; `route_server` optional.

## 4. Mecanum-specific configuration
| Component | Guidance |
|---|---|
| Footprint | polygon incl. forks (fork extends beyond chassis; consider footprint with/without load) - decide and record |
| DWB | holonomic requires `vy` sampling (`vy_samples`, `max_vel_y`/`min_vel_y` in the DWB plugin params - verify names in the installed version); critics tuned for strafing |
| MPPI | `motion_model: "Omni"` (not `DiffDrive`); set `vy_max`, `vy_min`, `wz_max`; verify param names against installed `nav2_mppi_controller` |
| RPP | commands vx and wz only - it will not exploit strafing; usable baseline, possibly with Rotation Shim |
| Velocity limits | vx, vy, wz max and acceleration; forklift carrying pallet needs lower limits (TBD) |
| Smoother/collision monitor | polygons based on the footprint; include lateral directions (strafe) |
Baseline order: **(1) NavfnPlanner + DWB (or RPP) -> (2) MPPI Omni -> (3) route graph**. Evaluate each against the same test set.

## 5. Costmaps
Local (odom frame, rolling window) and global (map frame) with obstacle layer from `/scan`, inflation radius from the footprint, static layer from the map. The reference repo adds a custom `nav2-virtual-layer` plugin for keep-out zones - a separate package that must be reviewed for Humble compatibility before reuse.

## 6. Behavior trees, route graph, GeoJSON
- BTs: start from Nav2's `navigate_to_pose_w_replanning_and_recovery.xml`; custom BT nodes (reference: `ConcatenatePaths`, `PublishPath`) only when needed.
- Route graph: `nav2_route` is installed; the reference uses a GeoJSON graph (`forklift_routeV2.geojson`) with `DistanceScorer`, `DynamicEdgesScorer` and operations `AdjustSpeedLimit`, `ReroutingService`. Verify the Humble 1.1.20 `nav2_route` API/params and GeoJSON schema from the installed package before designing the route file. Treat route handling as milestone 9b (after baseline).
- Recoveries: clear costmaps, spin, wait, backup - adapt `BackUp`; consider `DriveOnHeading` sideways is not available (check `behavior_server` plugins).

## 7. Tests
```bash
ros2 launch <nav launch>                 # bringup
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {...}}"
ros2 topic echo /cmd_vel_nav ; ros2 lifecycle nodes
```
Acceptance (M9): lifecycle nodes active; goal success rate over N runs (e.g. 10 start/goal pairs, N TBD) with final error <= TBD cm / TBD deg; strafe is used where intended (vy != 0 in `cmd_vel_nav`); obstacle appears -> replans or stops; goal cancel works; recovery triggers on blocked path; no collision in the test set. Record controller, params file revision, and results.

## 8. Failure modes
Controller rejects goal ("no valid trajectory"): footprint/inflation too large for aisle; oscillation: critic weights or too-high limits; robot refuses to strafe: controller or velocity smoother constrains `vy` to 0; "Failed to create plugin": wrong class name (check installed plugin xml); stuck because costmap frame is not `odom`/`map`.

## 9. Documentation
Params files with comments, the chosen baseline and why, controller comparison table, route graph spec, test log.
