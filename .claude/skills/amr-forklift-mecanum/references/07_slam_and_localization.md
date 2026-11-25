# 07 - SLAM and Localization

## 1. Purpose
Build maps (SLAM Toolbox), serve them (map server) and localize against them (AMCL), with correct TF and mecanum motion model.

## 2. Compatibility (verified 2026-11-02, this machine)
`ros-humble-slam-toolbox` 2.6.10, `nav2_amcl` 1.1.20, `nav2_map_server` (in Nav2 1.1.20), `nav2_lifecycle_manager`, `robot_localization` 3.5.4 are installed. Parameter names must still be checked against the installed package's default YAMLs (`/opt/ros/humble/share/slam_toolbox/config/`, `nav2_bringup/params/`).

## 3. Mapping workflow (M7) - done 2026-11-06, see docs/testing/2026-11-06_m7_slam_mapping.md and ADR-0005
Implemented in `mecanum_forklift_localization` (`config/slam_toolbox_mapping.yaml`, `launch/mapping.launch.py`, `maps/simple_warehouse.*`).
1. Sim world with sufficient features (`simple_warehouse.sdf`); lidar `/scan` (existing YDLidar X2 model; verify range, FOV, frame, rate matches the sensor tags).
2. Start sim + `odom->base_footprint` TF + robot_state_publisher.
3. `slam_toolbox` online async (reference repo uses `mapping_launch.py`; mode name `online_async`), params: `odom_frame`, `map_frame`, `base_frame`, `scan_topic`, `resolution`, `max_laser_range`, `use_sim_time`.
4. Drive with teleop slowly; mecanum strafe can smear scans if yaw drifts - map with moderate speeds and check loop closure.
5. Save: `ros2 run nav2_map_server map_saver_cli -f <name>` (writes .yaml + .pgm) into the navigation package `maps/`. Serialization for continued mapping (`serialize_map` service of slam_toolbox) optional.
Acceptance: map visually matches world geometry (measure two known distances, error <= TBD cm); map files saved and reloadable.

## 4. Localization workflow (M8) - done 2026-11-06, see docs/testing/2026-11-06_m8_amcl_localization.md and ADR-0006
Implemented in `mecanum_forklift_localization` (`config/amcl.yaml`, `launch/localization.launch.py`, `config/localization.rviz`). The OmniMotionModel is present in Humble Nav2 1.1.20 (`nav2_amcl/plugins.xml`); the differential model failed the mecanum tour.
- `map_server` + `amcl` under a lifecycle manager (`autostart`), initial pose via RViz or param.
- **Motion model**: the reference uses `DifferentialMotionModel`. For mecanum use `nav2_amcl::OmniMotionModel` (verify it is available in Humble Nav2 1.1.20: check `nav2_amcl` plugin xml / docs; param `robot_model_type`). Do not copy the diff model.
- Set `base_frame_id`, `odom_frame_id`, `global_frame_id`, `scan_topic`; particle count; `update_min_d/a`.
- If an EKF is used, AMCL consumes its `odom->base_footprint`.
Acceptance: after a known initial pose, `map->odom` converges; position error vs ground truth <= TBD after driving a loop; global localization recovers from a kidnapped-robot pose (optional); no TF warnings.

## 5. Tests
`ros2 lifecycle get /amcl`, `ros2 topic echo /amcl_pose`, compare to Gazebo ground truth, record RMS error. Repeat after rotation-only and strafe-only motion, since these stress the motion model.

## 6. Failure modes
Map skew (odom yaw drift); AMCL jumping (wrong motion model, too few particles, scan not matching map resolution); "Message Filter dropping" (TF timing); both SLAM and AMCL publishing `map->odom`; wrong scan frame orientation (lidar mounted rotated).

## 7. Documentation
Mapping runbook, saved map list with world/version, parameters with rationale, error measurements.

## 8. Open
Final map storage location and naming; whether 3D/pallet-level obstacles require a second sensor.
