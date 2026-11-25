# M9 - Nav2 navigation (simulation) - 2026-11-23

Stack: `mecanum_forklift_navigation` (Nav2 1.1.20 Humble: NavFn planner, SimpleSmoother, behaviors, bt_navigator with the stock
`navigate_to_pose_w_replanning_and_recovery` tree, velocity smoother -> twist_mux -> cmd_vel_gate), AMCL localization, curated nav map,
footprint incl. fork tines (x -0.205..0.368, y +-0.154), limits 0.3 m/s / 1.0 rad/s (sim placeholders). Start: spawn pose, AMCL initial pose (0,0,0).
Tools: `scripts/nav_trials.py` (goal chain, ground-truth error, clearance, strafing, AMCL error, objects pushed) and `scripts/nav_failures.py`.
Ground truth = Gazebo `/odom_ground_truth`; real-time factor ~0.4-0.7, times are wall-clock seconds.

## Smoke test (before any goal)
All 10 lifecycle nodes active, no errors; `map -> base_link` present; planning-only requests succeeded to four staging poses and failed for a goal inside a wall.

## Goal set (10 chained goals, each starts where the previous ended)
1 strafe -1.0 m; 2 forward 1.2 m; 3 strafe +2.0 m; 4 move + turn 180 deg; 5 between the west cones; 6 facing pallet_2 (turn 180 deg in the cone aisle);
7 staging pose pallet_1; 8 staging pose pickup pallet; 9 staging pose charger 1; 10 back to start. Goal feasibility checked against the map (>= 20 cm footprint clearance).

## Controller comparison (same goals)
| run | controller / change | succeeded | within 5 cm / 5 deg | collisions | notes |
|---|---|---|---|---|---|
| 1 | DWB baseline (BaseObstacle critic, goal tol 0.1 m / 0.1 rad) | 9/10 | 0/10 | 2 (cone, then AMCL lost: 1.9 m error) | goal 6 timed out; stopped at the tolerance edge (5-7 cm, 5.5 deg) |
| 2 | + ObstacleFootprint critic, goal tol 0.03 m / 0.05 rad | (stopped) | - | - | goal 1 took 194 s (dithering at the goal), run aborted |
| 3 | goal tol 0.05 m / 0.08 rad | 9/10 | 4/10 | 1 map touch (no object moved) | goal 6 timed out |
| 4 | + progress checker 0.10 m / 30 s | 9/10 | 5/10 | 1 map touch (no object moved) | goal 6 still stuck (see below); vy > 0.05 m/s only 17-25 % of the strafe goals |
| 5 | **MPPI Omni**, goal tol 0.1 m / 0.1 rad | 10/10 | 3/10 | 0 | 9-35 s per goal, strafing in 95-99 % of the strafe goals |
| 6 | **MPPI Omni, goal tol 0.04 m / 0.07 rad** | 10/10 | 9/10 | 0 | |
| 7 | same, repeat | 10/10 | 8/10 | 0 | |

**MPPI Omni (runs 6+7, N = 20 goals): 20/20 succeeded, 17/20 within 5 cm / 5 deg, 0 collisions, 0 objects pushed.** Final error median 3.0 cm, max 6.8 cm
(goal 4 in both runs: 6.8 / 6.5 cm; goal 1 once 5.1 cm); yaw max 3.6 deg; 9-45 s per goal (median 19 s); AMCL error vs ground truth <= 73 mm throughout;
sideways speed reaches 0.30 m/s and is above 0.05 m/s for 95-99 % of the samples in the two strafe goals. The 3 misses are 0.1-1.8 cm over the 5 cm criterion.

## Findings (causes verified from logs / DWB evaluation topic unless stated)
- **DWB BaseObstacle critic only checks the robot centre**: the 0.37 m tines swept into cones while turning (2 collisions + AMCL lost in run 1). `ObstacleFootprint` fixed it.
- **"Failed to make progress"**: the default progress checker (0.15 m / 10 s) aborts a 180 deg turn in place; now 0.10 m / 30 s in all controller files.
- **DWB local minimum (goal 6)**: with the robot at (-1.77, 0.06), the zero-velocity trajectory scored 40.92 and the best translating trajectory 40.93 (path-following terms ~19 cancel the goal-distance gain ~20); the robot never moves. Not tuned further: MPPI does not have the problem.
- **Goal tolerance vs localization**: 3 cm goal tolerance makes DWB dither (194 s for 1 m); AMCL itself is 3-7 cm off ground truth, so the 5 cm criterion is near the localization limit.
- The painted nav-map cells include a ~2.5 cm margin, so "map touch" is stricter than a real collision; real contact is measured by objects moved in Gazebo.

## Failure injection (MPPI)
| case | result |
|---|---|
| cancel mid-goal | PASS: CANCELED, speed < 2 cm/s after 0.34 s wall, 0.0 mm drift afterwards |
| teleop during navigation | PASS: robot follows teleop (-0.1 m/s) while the goal is active, the goal then succeeds |
| 0.5 m tall box dropped on the path | PASS: replanned, SUCCEEDED, no contact |
| goal deep inside shelf_2 | PASS: drives 33 cm toward the nearest reachable point, ABORTED after 84 s, stopped |
| goal on a box dropped onto it, then box removed | PASS with a gap: Nav2 did not give up within 325 s (the test cancelled, robot stopped); the same goal succeeded after removing the box |
| goal in the pallet_1 centre | not a rejection: NavFn tolerance 0.1 m plans to the nearest free spot (the pallet centre is ~0.1 m from free space) |

## Not done / open
- **No bounded give-up time** for a goal blocked after planning (default BT retries without an overall limit): the mission manager (M12) must apply navigation timeouts and cancel.
- Collision monitor not inserted (deferred, as agreed); route graph (9b) not done; footprint tested unloaded only (a carried pallet adds ~0.07 m at the front).
- N = 20 goals on one world and one start sequence; no random start poses; DWB goal 6 not fixed; the pallet_1 staging goal was reached at the 0.165 m pose only for navigation, not combined with docking (M11c).
- Limits (0.3 m/s, 1.0 rad/s) are sim placeholders. Simulation does not establish physical safety.
