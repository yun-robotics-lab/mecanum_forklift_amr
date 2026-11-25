# mecanum_forklift_control

Velocity path and odometry for the 4-wheel mecanum base (simulation drive is the Gazebo `MecanumDrive` plugin, ADR-0002). Odometry ownership: ADR-0004.

## Nodes
| Node | What | In -> Out |
|---|---|---|
| `twist_mux` (external) | arbitration, teleop priority 100 over navigation priority 10 | `cmd_vel_teleop`, `cmd_vel_nav_safe` -> `cmd_vel_mux` |
| `cmd_vel_gate` | watchdog (0.3 s wall clock -> zero) and speed clamp; only publisher of `cmd_vel` | `cmd_vel_mux` -> `cmd_vel` |
| `wheel_odometry` | integrates wheel joint states; owns TF `odom -> base_footprint` | `joint_states` -> `odom`, `/tf` |

Also: `mecanum_forklift_control/kinematics.py` (forward/inverse kinematics, pose integration).

## Files
`config/control.yaml` (gate limits, wheel geometry; must match the description), `config/twist_mux.yaml` (sources, priorities, timeouts), `launch/control.launch.py` (arg `use_sim_time`, default true).

## Run
Included by the Gazebo launch. Standalone: `ros2 launch mecanum_forklift_control control.launch.py`.
```bash
ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}"   # drives; stops 0.3 s after you stop publishing
ros2 topic echo /odom
```

## Test
```bash
colcon build --packages-select mecanum_forklift_control
colcon test --packages-select mecanum_forklift_control && colcon test-result --verbose
```
Unit tests: `test_kinematics.py`, `test_cmd_vel_gate.py` (+ flake8, pep257). Drive behaviour (forward, strafe, rotate, circles, stop time, odometry versus ground truth) was tested in simulation: `docs/testing/2026-11-06_m5_teleop_safety.md` and `2026-11-06_m6_odometry_tf.md`. Any change to drive or kinematics needs the strafe and rotate tests again.

## Developing
- Keep `wheel_radius`, `wheel_separation` and `wheelbase` equal to the description values.
- Exactly one publisher of `odom -> base_footprint` (this node); do not add an EKF or simulator TF.
- Speed limits (0.6 / 0.6 / 2.0) and covariances (0.01) are sim placeholders; confirm on hardware.
- A teleop source that keeps publishing zero Twists (the keyboard node does) blocks navigation in the mux.

## Update parameters
Edit `config/control.yaml` (`cmd_vel_gate`: speed limits, watchdog; `wheel_odometry`: wheel geometry, covariances) or `config/twist_mux.yaml` (topics, priorities, timeouts; e-stop 200 > teleop 100 > docking 50 > navigation 10). Node keys use the wildcard form `/**/<node>:` so the same file works in a namespace. Rebuild (`colcon build --packages-select mecanum_forklift_control`) and relaunch; live: `ros2 param set /cmd_vel_gate <name> <value>`.

## Multi-robot
Run under a namespace (`/robot1/...`) with `/tf` remapped to `tf`; `mecanum_forklift_bringup` does this per robot. All topic names here are relative.
