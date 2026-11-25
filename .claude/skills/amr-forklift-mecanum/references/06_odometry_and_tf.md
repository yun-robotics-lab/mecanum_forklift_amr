# 06 - Odometry and TF

## 1. Purpose
Define who publishes which transform and how odometry is produced, fused and validated. Most navigation failures are TF-ownership failures.

## 2. Frame ownership (proposed - finalize after implementation, record in ADR)
| Transform | Owner | Notes |
|---|---|---|
| `map -> odom` | AMCL (localization) or SLAM Toolbox (mapping) | never both at once |
| `odom -> base_footprint` | **exactly one of**: drive odometry (gz plugin / `mecanum_drive_controller`) **or** `robot_localization` EKF | if EKF owns it, set the drive's TF publishing off (`enable_odom_tf: false` for the controller; for the Gazebo plugin remove/redirect `tf_topic`) |
| `base_footprint -> base_link` | `robot_state_publisher` (fixed joint) | |
| `base_link -> wheel/fork/sensor links` | `robot_state_publisher` from URDF + `/joint_states` | fork and wheels are dynamic joints |

Current state (tested 2026-11-06, ADR-0004): the Gazebo `MecanumDrive` plugin does NOT publish odometry. `/odom` and `odom->base_footprint` come from `mecanum_forklift_control/wheel_odometry` (50 Hz, joint-increment integration); Gazebo `OdometryPublisher` provides ground truth on `/odom_ground_truth` with TF on the unbridged `/ground_truth/tf`. IMU `/imu` (frame `imu_link`) exists but no EKF is used (no measurable gain in sim). See docs/testing/2026-11-06_m6_odometry_tf.md.

## 3. Wheel odometry
Integrate body velocity from wheel velocities (forward kinematics, 04) in the body frame, rotate into odom. Mecanum odometry is **less accurate than differential** (roller slip in strafe): expect vy and yaw drift; covariances must reflect it (larger vy/yaw variance). Simulated odometry from Gazebo plugins may be near-ideal and hide this - do not use simulated accuracy to size covariances.

## 4. IMU and EKF
- `robot_localization` 3.5.4 installed (ekf_node). The reference `ekf.yaml` fuses `/odom` + `/imu` and uses it as an optional layer.
- Typical planar config: `two_d_mode: true`, fuse vx, vy, wz from odom; yaw rate (and optionally yaw) from IMU; `publish_tf` per ownership table; `world_frame: odom`.
- IMU now exists (2026-11-06): `imu_link`, 100 Hz `/imu`, world `Imu` system, bridge entry. `imu_link` needs a tiny inertial or Gazebo drops the sensor.
- Decision gate: only add the EKF if it measurably improves yaw/position vs ground truth in sim (and is justified for hardware); record the comparison.

## 5. Validation
```bash
ros2 run tf2_tools view_frames            # one connected tree, one parent per frame
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /odom ; ros2 topic echo /odom --once
ros2 topic info /tf -v                    # who publishes
```
Acceptance (M6): single connected tree; no duplicate publishers for any child frame; `odom` timestamps use sim time; `/odom` twist frame is `base_footprint`/`base_link` as documented; straight-line, strafe and rotation tests (04) match ground truth within approved tolerance; static robot shows no drift beyond TBD mm/min.

## 6. Common problems
| Symptom | Cause |
|---|---|
| TF jumps / flicker | two publishers for `odom->base_footprint` (plugin + EKF) |
| "extrapolation into the future/past" | `use_sim_time` mismatch, bridge latency, missing `/clock` |
| Robot rotates twice as fast in odom | wrong L, wrong wheel radius, double-counted gear ratio |
| Drift while stationary | IMU bias in EKF, noisy wheel velocity at standstill |
| Odom frame in `/tf` but Nav2 says no transform | `base_frame` param name mismatch (`base_footprint` vs `base_link`) |
| Multi-robot TF collisions | shared `/tf` and unprefixed frames (see 14) |

## 7. Documentation
Final frame table, TF tree image from `view_frames`, EKF YAML, covariance rationale, test numbers.
