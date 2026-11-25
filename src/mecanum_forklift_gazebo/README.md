# mecanum_forklift_gazebo

Gazebo Fortress (`ign`) simulation: worlds, models, ROS-Gazebo bridge and the launch that starts the whole simulated robot (Gazebo, `robot_state_publisher`, spawn, bridge, control, lift controller).

## Files
| File | What |
|---|---|
| `launch/mecanum_forklift_gazebo.launch.py` | main launch; includes `mecanum_forklift_control` and `mecanum_forklift_lift` launches |
| `config/bridge_config.yaml` | bridged topics: `/clock`, `/cmd_vel`, `/odom_ground_truth`, `/joint_states`, `/fork/cmd_pos`, `/scan`, `/camera/image_raw`, `/camera/camera_info`, `/imu` |
| `worlds/simple_warehouse.sdf` (default), `worlds/empty.sdf` | worlds |
| `worlds/warehouse_v2.sdf` | 8 x 6 m, two rack bays (level at 0.060 m), floor pick and drop, two moving cylinders. No maps or dock entries yet |
| `scripts/obstacle_mover.py`, `config/dynamic_obstacles.yaml`, `config/obstacle_bridge.yaml` | cylinder motion along x and y (amplitude, speed) |
| `models/` | cargo_box, cargo_box_marked (box + marker 0 joined), charging_station, dropoff_station, mini_pallet, pickup_station, safety_cone, shelf |

Launch arguments: `world` (default `simple_warehouse.sdf`), `x y z yaw` (spawn pose), `gui` (default true), `use_sim_time` (default true), `dynamic_obstacles` (default false; `warehouse_v2.sdf` only).

New world with moving cylinders: `ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py world:=warehouse_v2.sdf dynamic_obstacles:=true`
The Gazebo TF is not bridged on purpose; `wheel_odometry` owns `odom -> base_footprint`.

## Run
```bash
cd ~/Personal_Project/mecanum_forklift_amr
colcon build && source install/setup.bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false     # headless
ros2 topic list; ros2 topic hz /scan; ros2 topic echo /odom --once
```
Stop with Ctrl-C and check no stale `ign gazebo`, `twist_mux`, `lift_controller` processes remain (`ps -eo pid,cmd | grep -E "ign gazebo|twist_mux"`); stale ones cause duplicate nodes and confusing results.

## Test
`colcon test --packages-select mecanum_forklift_gazebo` runs lint only. Simulation checks (spawn, static stability 60 s, topic rates, real-time factor 0.4-0.7) are in `docs/testing/2026-11-06_m3-4_cleanup_step_b.md`. Pallets and boxes are below the 0.224 m lidar plane and invisible to it.

## Developing
- New sensor: add the plugin/sensor in the description's `gazebo.xacro`, then a bridge entry here, then check the topic rate.
- Use `ignition-gazebo-*` plugin names (Fortress), never `gz-*`.
- The bridge `/joint_states` runs near 700 Hz; throttle if it becomes a load problem.
- Simulation does not validate physical safety.

## Update parameters
Launch arguments (`world`, `gui`, `use_sim_time`, spawn pose) are listed by `ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py --show-args`. Bridged topics are in `config/bridge_config.yaml` (`topic@ros_type[gz_type`: `[` = Gazebo to ROS, `]` = ROS to Gazebo). Edit, rebuild (`colcon build --packages-select mecanum_forklift_gazebo`), relaunch. For several robots use `mecanum_forklift_bringup` (per-robot bridges and namespaces) instead of this launch file.
