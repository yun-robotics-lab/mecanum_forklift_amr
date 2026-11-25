# 02 - Robot Description (URDF/Xacro)

## 1. Purpose
Rules and checklist for the robot model in `src/mecanum_forklift_description`. The model is the source of truth for geometry, TF, inertia, sensor frames and (later) ros2_control joints.

## 2. Current state (inspected 2026-11-02, unverified)
- Main file `urdf/mecanum_forklift.urdf.xacro` (152 lines) includes `inertial_macros.xacro`, `mecanum_wheel.xacro`, and `mecanum_forklift.gazebo.xacro`.
- Links: `base_footprint` (empty) -> fixed -> `base_link`; wheels `fl|fr|rl|rr_wheel_link` (continuous, axis `0 1 0`); `fork_link` via prismatic `fork_joint` (max 0.12 m via `fork_lift_max`); `lidar_link`; `front_camera_link` (mounted on the fork carriage, moves with the fork).
- Wheel placement: x = +-0.11 m, y = +-(base_width/2 + wheel_width/2 - 0.02) = +-0.1305 m, radius 0.039 m, width 0.047 m, mass 0.3 kg.
- Wheel collision is a **sphere** (deliberate, so Gazebo friction directions work; see 03/04). Visual is the STL mesh.
- Meshes use `package://mecanum_forklift_description/meshes/...` (good). STL scale 0.001 (mm).
- `material.xacro` now holds the colours; the empty `gazebo.xacro`/`ros2_control.xacro` placeholders no longer exist. Added 2026-11-06: `front_camera_optical_frame` (rpy -pi/2 0 -pi/2) and `imu_link` (+ `/imu` sensor, world Imu system, bridge entry). `imu_link` needs a tiny inertial or Gazebo drops the sensor.
- Dimensions comment: base 407 x 254 x 204 mm, mass 5.0 kg - this is a **small-scale** robot; confirm intended scale (Q in 01).

## 3. Target Xacro organisation
```
urdf/
  mecanum_forklift.urdf.xacro   main: includes + assembly + args
  materials.xacro               colours only
  constants.xacro               dimensions, masses, limits (xacro:property)
  inertial_macros.xacro         box/cylinder/sphere inertia macros
  chassis.xacro   wheels (mecanum_wheel.xacro)   fork.xacro
  sensors.xacro                 lidar, imu, camera frames
  gazebo.xacro                  <gazebo> tags: plugins, friction, sensors
  ros2_control.xacro            only if ADR-0002 selects ros2_control
```
Use xacro arguments for: `namespace`/`prefix` (multi-robot), `use_sim`, `sim_backend`.

## 4. Frames (proposed; final in 06)
`base_footprint` (ground projection, odom child) -> `base_link` (chassis center) -> `{fl,fr,rl,rr}_wheel_link`, `fork_link`, `lidar_link`, `imu_link`, `front_camera_link`(+optical frame). REP-103: x forward, y left, z up. Camera optical frames need a separate rotated frame (REP-103 optical convention) if the camera driver/pose estimation expects it - verify per consumer.

## 5. Validation checklist
```bash
xacro src/mecanum_forklift_description/urdf/mecanum_forklift.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf            # needs liburdfdom-tools (check if installed; ask before installing)
ros2 launch mecanum_forklift_description display.launch.py   # RViz + joint_state_publisher_gui
ros2 run tf2_tools view_frames
```
- Mass > 0 and a positive-definite inertia for every moving link; inertia magnitudes plausible (not 1e-9 or 1e3).
- Collision geometry simple (primitives); no huge mesh colliders for wheels.
- Joint origins: wheel axes parallel to Y and symmetric about base_link; wheel z so that wheels touch ground with `base_footprint` at z=0.
- Fork joint limits (lower/upper/effort/velocity) set and consistent with 10.
- No absolute paths: `grep -rn "/home/" src/mecanum_forklift_description` must be empty (note: `.vscode/settings.json` in project root contains one - editor config, not a robot file).
- Sensor frames match topic `frame_id` values emitted in Gazebo.

## 6. Failure modes
- Robot "explodes"/jitters in Gazebo: bad inertia, huge mesh collision, stiff contact params.
- Wheels spin but no motion: friction tags lost in URDF->SDF, mesh collision ignores `mu` directions, sphere collision radius mismatch.
- RViz robot white / missing: mesh path or material.
- `preserveFixedJoint` missing -> fixed-joint links merged in SDF, sensor frame disappears.

## 7. Documentation requirements
Record in `docs/development/`: frame table, dimension table with source (CAD/measurement), inertia derivation, changes to the model and the validation output.

## 8. Unverified / open
Real robot dimensions and mass; whether `base_footprint` should be the odom child (current gz plugin config uses it).
