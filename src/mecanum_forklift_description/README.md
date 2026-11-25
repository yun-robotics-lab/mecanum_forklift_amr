# mecanum_forklift_description

Robot model: modular Xacro (chassis, 4 mecanum wheels, fork on a prismatic joint, YDLidar X2, front camera, IMU) plus the Gazebo Fortress plugin block. `package://` mesh paths only.

## Files
| File | What |
|---|---|
| `urdf/mecanum_forklift.urdf.xacro` | main model: links, joints, sensors, constants (`fork_lift_max` 0.12 m, wheel geometry) |
| `urdf/mecanum_wheel.xacro` | wheel macro (hub, rollers, axle); `flip` selects roller orientation |
| `urdf/mecanum_forklift.gazebo.xacro` | Gazebo plugins: `MecanumDrive`, `OdometryPublisher` (ground truth only), `JointStatePublisher`, fork `JointPositionController` (PID 5000/50/150), lidar, camera, IMU |
| `urdf/inertial_macros.xacro`, `material.xacro` | inertia macros, materials |
| `meshes/`, `rviz/view_robot.rviz`, `launch/display.launch.py` | meshes, RViz config, RViz viewer (arg `gui`, default true) |

Frames: `base_footprint -> base_link -> {wheels, fork_link -> front_camera_link, lidar_link, imu_link}`; see `docs/architecture/` for the full table.

## Run and validate
```bash
cd ~/Personal_Project/mecanum_forklift_amr
colcon build --packages-select mecanum_forklift_description && source install/setup.bash
xacro src/mecanum_forklift_description/urdf/mecanum_forklift.urdf.xacro > /tmp/robot.urdf && check_urdf /tmp/robot.urdf
ros2 launch mecanum_forklift_description display.launch.py          # RViz + joint sliders
```
Last full check: `docs/testing/2026-11-06_m3-4_cleanup_step_b.md`. No automated tests besides lint.

## Developing
- After editing any Xacro, rebuild this package (the Gazebo launch reads the installed copy).
- Wheel/odometry geometry here must match `mecanum_forklift_control/config/control.yaml`.
- Changing roller orientation or joint axes needs the strafe and rotate tests.
- Fork joint limits must match the lift controller limits; the fork rests at -0.001 m (cause unknown).
- URDF to SDF conversion drops some tags: check plugins in the generated SDF after changes.

## Xacro arguments
| Argument | Default | Meaning |
|---|---|---|
| `gz_ns` | `""` | prefix of all Gazebo topics (e.g. `/robot1`); used for multi-robot, empty = unchanged single robot |
| `sensor_profile` | `full` | `light`: lidar 180 samples at 5 Hz and no camera sensor (multi-robot on a CPU-only machine) |

```bash
xacro urdf/mecanum_forklift.urdf.xacro gz_ns:=/robot1 sensor_profile:=light > /tmp/r1.urdf && check_urdf /tmp/r1.urdf
```
To change dimensions, masses, sensor rates or PID gains edit the files in `urdf/`, rebuild (`colcon build --packages-select mecanum_forklift_description`) and re-run the validation above; control parameters that depend on wheel geometry live in `mecanum_forklift_control/config/control.yaml`.
