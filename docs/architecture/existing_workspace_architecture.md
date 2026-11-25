# Existing workspace architecture

Snapshot of what is in `src/` on 2026-11-05 (read from source; behaviour evidence is in `docs/testing/existing_package_test_report.md`). Planned architecture stays in `.claude/skills/amr-forklift-mecanum/references/01_project_architecture.md`.

## Packages

| Package | Build type | Contents | Depends on (package.xml) |
|---|---|---|---|
| `mecanum_forklift_description` | ament_cmake | Xacro model, 6 meshes (STL/DAE), RViz config, `display.launch.py` | robot_state_publisher, joint_state_publisher(+gui), xacro, rviz2, ros_gz_sim, ros_gz_bridge, launch(_ros) |
| `mecanum_forklift_gazebo` | ament_cmake | `launch/mecanum_forklift_gazebo.launch.py`, `config/bridge_config.yaml`, 2 worlds, 7 models | description, ros_gz_sim, ros_gz_bridge, robot_state_publisher, xacro, launch(_ros) |
| `mecanum_forklift_control` | ament_python | `cmd_vel_gate`, `wheel_odometry`, `control.yaml`, `control.launch.py`, unit tests | rclpy, geometry_msgs, nav_msgs, sensor_msgs, tf2_ros |
| `mecanum_forklift_teleop` | ament_python | `keyboard_node`, `ps4_node`, `ps4.yaml`, `ps4_teleop.launch.py` | rclpy, geometry_msgs, std_msgs, sensor_msgs, joy |

## Robot model (`urdf/`)

`mecanum_forklift.urdf.xacro` is the only entry point; it includes `inertial_macros.xacro`, `mecanum_wheel.xacro` and `mecanum_forklift.gazebo.xacro`. Materials are defined inline. **`gazebo.xacro`, `material.xacro`, `ros2_control.xacro` are empty (0 bytes) and not included anywhere.**

Link/joint tree (confirmed by `check_urdf`):

```
base_footprint
└─ base_link              (fixed, z = 0.02)
   ├─ fl/fr/rl/rr_wheel_link   continuous, axis (0 1 0), parent base_link
   ├─ fork_link                prismatic z, 0..0.12 m, effort 100, vel 0.1
   │  └─ front_camera_link     fixed
   └─ lidar_link               fixed, at base top (z = 0.204)
```

Key numbers: base 0.407 x 0.254 x 0.204 m, 5 kg; wheel r = 0.039 m, width 0.047, 0.3 kg; wheel x = ±0.11, y = ±0.1305 (wheel_separation 0.261, wheelbase 0.22 in the plugin); fork 0.5 kg. Wheel collision is a sphere (deliberate, documented in the xacro: needed for anisotropic friction).

## Gazebo layer (`mecanum_forklift.gazebo.xacro`, Fortress)

- Wheel friction: `mu1=1, mu2=0`, `fdir1` per wheel (fl `1 -1 0`, fr `1 1 0`, rl `1 1 0`, rr `1 -1 0`) = X roller pattern.
- Systems in the robot: `MecanumDrive` (topic `cmd_vel`; no odometry, see below), `OdometryPublisher` (ground-truth `/odom` + `odom->base_footprint` TF, frames `odom`/`base_footprint`, 50 Hz, 2D), `JointStatePublisher` (`joint_states`), `JointPositionController` on `fork_joint` (`fork/cmd_pos`).
- Sensors: `gpu_lidar` `/scan` (360 samples, 0.12-8 m, 10 Hz) on `lidar_link`; `camera` `/camera/image_raw` + `camera_info` (640x480, 30 Hz) on `front_camera_link`.
- World-level systems (physics, sensors, scene broadcaster, user commands) are in `worlds/*.sdf` (`mecanum_world`, 1 ms step).

## Launch / data flow (`mecanum_forklift_gazebo.launch.py`)

1. Sets `IGN_GAZEBO_RESOURCE_PATH` (workspace share parent + `models/`) so `model://mecanum_forklift_description/meshes/...` resolves.
2. Includes `ros_gz_sim/gz_sim.launch.py` with `-r -v 3 <world>`; arg `gui` (default `true`; `false` adds `-s`, server only).
3. `robot_state_publisher` from the xacro (`use_sim_time` launch arg, default `true`).
4. At launch time converts URDF -> SDF with `ign sdf -p`, re-adds `ignition:expressed_in` to every `<fdir1>`, writes both to a new `/tmp/mecanum_forklift_*` dir and spawns it with `ros_gz_sim create`.
5. `ros_gz_bridge parameter_bridge` from `bridge_config.yaml`.

```
teleop (keyboard_node | ps4_node) --/cmd_vel_teleop--> cmd_vel_gate --/cmd_vel--> [bridge] --> Gazebo MecanumDrive --> wheel joints
                                  --/fork/cmd_pos--> [bridge] --> JointPositionController
Gazebo --> [bridge] --> /scan /camera/* /odom /tf /joint_states /clock
/joint_states --> robot_state_publisher --> /tf, /tf_static
/joint_states --> wheel_odometry --> /odom + TF odom->base_footprint
Gazebo OdometryPublisher --> /odom_ground_truth (test reference only; its TF is not bridged)
```

Bridged topics: `/clock`, `/cmd_vel`, `/odom_ground_truth`, `/joint_states`, `/fork/cmd_pos`, `/scan`, `/camera/image_raw`, `/camera/camera_info`.

## Teleop

- `keyboard_node`: latched direction keys (w/x fwd/back, a/d strafe, q/e rotate, space/k/s stop), fork r/f/v, speed t/g/y/h; publishes `cmd_vel` at 20 Hz and `fork/cmd_pos` on key press. Needs a TTY (`ros2 run`, not launch). No timeout.
- `ps4_node`: subscribes `joy`; L1 deadman, left stick vx/vy, right stick X wz, R1 turbo, D-pad scale, face buttons fork. Parameters in `ps4.yaml`. Publishes one zero Twist on L1 release.

## Control and TF ownership as built

- Control: **Gazebo `MecanumDrive` plugin only** (open ADR-0002). No ros2_control, no controller YAML, no `controller_manager`, no arbitration node.
- TF owners as implemented: `base_footprint->base_link->sensors/wheels/fork` = `robot_state_publisher`; `odom->base_footprint` = `wheel_odometry` (single owner); `map->odom` none.

Note (2026-11-05): Fortress 6.18 `MecanumDrive` never publishes odometry (verified from the plugin binary), which is why `OdometryPublisher` is used. `MecanumDrive` also latches the last `cmd_vel` with no timeout.

Update 2026-11-05 (control package): velocity chain is now `cmd_vel_teleop -> cmd_vel_gate -> cmd_vel` (watchdog 0.5 s wall clock, clamp 0.6/0.6/2.0, single publisher of `cmd_vel`). The sim launch includes `mecanum_forklift_control/launch/control.launch.py`. The planned arbitration node replaces the gate's single input later.
