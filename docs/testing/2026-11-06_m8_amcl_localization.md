# 2026-11-06 - Milestone 8, AMCL localization

Setup: headless `mecanum_forklift_gazebo.launch.py gui:=false` (simple_warehouse), `mecanum_forklift_localization/launch/localization.launch.py rviz:=false` (map_server + nav2_amcl 1.1.20 + lifecycle manager, map `maps/simple_warehouse.yaml` with `free_thresh` changed 0.25 -> 0.196), RTF ~0.5. Error = TF `map->base_footprint` (AMCL + wheel odometry) versus the Gazebo model pose; map frame = world frame. Scripts were run from a scratch dir. Tour = the milestone 7 scripted tour (waypoints at 0.2 m/s with strafing, backward driving, 360 deg rotations both ways, second lap).

## Results
| Check | Result | Status |
|---|---|---|
| A. Known start, **first config** (500-2000 particles, `sigma_hit` 0.2, 60 beams, `update_min_d/a` 0.1) | mean 46.8 mm, worst 116.8 mm, worst yaw 2.65 deg (RMS not printed in that run) vs criteria RMS <= 50 mm, worst <= 100 mm, yaw <= 2 deg | tested, **not met** |
| A. Known start, **tuned config** (1000-5000 particles, `sigma_hit` 0.1, 120 beams, `update_min_d/a` 0.05), run 1 | RMS 34.3 mm / 0.25 deg; worst 70.0 mm / 1.65 deg | tested, met |
| A. same config, run 2 | RMS 33.1 mm / 0.25 deg; worst 64.9 mm / 1.65 deg | tested, met |
| B. Wrong start 0.5 m / 30 deg (RViz-like covariance 0.25 / 0.0685), clean baseline | error spikes to 409 mm / 28.5 deg, back within 10 cm / 3 deg from 3.2 s of driving and stays (criterion 30 s) | tested, met |
| B'. Wrong start 1.0 m + 0.5 m / 90 deg (outside the criterion) | **never recovered** within 53 s of driving, including a 360 deg rotation (error 0.6-2.5 m, ~90 deg) | tested, plain AMCL cannot recover from a gross initial error |
| C. Motion model, same tour with `DifferentialMotionModel` | RMS 526.9 mm / 26.15 deg, worst 2003 mm / 148 deg (diverged during the strafing and recovered later) vs omni 33-34 mm / 0.25 deg | tested, omni model required |
| D. `/reinitialize_global_localization`, one trial | error spiked to 339 mm / 86.5 deg, then converged to the true pose (not the mirrored one) within 3.0 s of driving | tested (informative, single trial) |
| E. TF owners and logs | `/tf` publishers: `amcl`, `robot_state_publisher`, `wheel_odometry`; `slam_toolbox` not running; 0 jump-back / extrapolation / message-filter warnings in sim and AMCL logs | tested |

## Tuning record
The first configuration came from the Nav2 defaults with `OmniMotionModel`, the sensor range and smaller update thresholds. It failed criterion A. I then changed `sigma_hit`, `max_beams`, particle counts and update thresholds in one step (no ablation, so I cannot say which change mattered) and re-ran. The criteria were set before any run; the tuned numbers meet them.

## Caveats
- The sim has almost no odometry noise, so AMCL's job is easy; the result says little about hardware. Hardware localization is unverified. Wheel slip and lidar noise are not modelled realistically.
- Test D is one trial; AMCL is stochastic. The room is nearly 180-degree symmetric (the shelf posts on the north side break the symmetry); do not assume global localization always lands on the right side.
- Several early B/D attempts were invalid for test-script reasons (initial pose published before the subscription matched; "at rest" samples read a stale transform because AMCL only updates after the robot moves; a start state left stuck from a previous gross-error run). The numbers above come from runs after those were fixed.
- Pallets and boxes are not in the map or in the lidar plane (0.224 m), same as milestone 7.
- Simulation does not validate physical safety.
