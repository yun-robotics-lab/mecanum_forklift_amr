# 2026-11-06 - Milestones 3/4 cleanup, Step B tests

Setup: `ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false` (simple_warehouse, includes control launch), headless, Ubuntu 22.04 / Humble / Fortress 6.18. Scripts were run from a scratch dir (not stored in the repo).

| Check | Result | Status |
|---|---|---|
| `xacro` + `check_urdf`, no `/home/` in description/gazebo | clean | tested |
| TF tree (`view_frames`) | one parent per frame; `imu_link`, `front_camera_optical_frame` present; `odom->base_footprint` single publisher | tested |
| Frame ids on topics | `/imu` = `imu_link`; `/camera/image_raw` = `front_camera_optical_frame` | tested |
| RTF (20 s sample) | 0.59 | measured |
| Rates, sim time | `/scan` 10.1 Hz, camera 29.5 Hz, `/imu` 100.0 Hz, `/odom` 47.9 Hz (cfg 50), `/joint_states` ~1000 Hz | tested |
| Rates, wall time (RTF 0.59) | scan 5.9, camera 17.2, imu 58.5, odom 28.0 Hz | measured |
| 60 s sim-time static | ground-truth drift 0.000 mm / 0.0000 deg; wheel odom 0; fork constant at -0.00100 m | tested |
| Motion regression (ground truth, body frame) | fwd 0.2 m/s x 5 s -> dx 1.000; strafe L -> dy 1.001; strafe R 3 s -> dy -0.600; diag 4 s -> (0.800, 0.800); rotate 0.5 rad/s x 4 s -> 114.6 deg; cross-axis error <= 1 mm, yaw 0.0 deg | tested |

Notes
- Fork rests at -0.001 m (1 mm below its lower limit) and does not drift. Treated as joint-limit compliance; cause not isolated.
- `/joint_states` publishes every physics step (~1 kHz sim time); consider throttling.
- Wall-clock sensor rates fall with the RTF (0.59). Sim-time rates are nominal. Acceptance criteria were defined in sim time.
- Not run: GUI/rendering check, GPU load, real hardware. Simulation does not validate physical safety.
- Mistake during testing: a second control launch duplicated nodes (the gazebo launch already includes control) and stale nodes from earlier runs were found; all were killed and the numbers above come from a clean single-instance run.
