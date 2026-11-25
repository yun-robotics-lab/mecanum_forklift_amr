# 03 - Gazebo Fortress Simulation

## 1. Purpose
How the robot is simulated with **Gazebo Fortress (Ignition Gazebo 6)** on ROS 2 Humble, and the traps that come from mixing Fortress with newer Gazebo documentation.

## 2. Compatibility (verified on this machine, 2026-11-02)
| Item | Result |
|---|---|
| Simulator | `ign gazebo --versions` -> 6.18.0 (CLI is `ign`, **not** `gz sim`; `gz sim` is rejected here) |
| ROS bridge | `ros-humble-ros-gz-sim` / `ros-humble-ros-gz-bridge` 0.244.24 installed (this is the Fortress-compatible ros_gz line for Humble) |
| System plugins present | `libignition-gazebo6-{mecanum-drive,odometry-publisher,imu,sensors}-system.so` under `/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins`; JointPositionController/JointStatePublisher are used by the existing model (library presence not separately checked) |
| ros2_control in Gazebo | `ros-humble-ign-ros2-control` 0.7.21 is an apt **candidate, not installed**; it is a transitional package depending on `ros-humble-gz-ros2-control`, which depends on `libignition-gazebo6`. `ros-humble-ros2-control` meta-package also not installed (controller_manager 2.53.1 and ros2_controllers 2.52.0 are). Installing needs user approval; check version skew (candidate controller_manager 2.54.3 vs installed 2.53.1). |

Naming rules for Fortress: plugin filenames `ignition-gazebo-*-system`, class names `ignition::gazebo::systems::*`, topics via `ignition.transport`, SDF `<world>` systems must be declared in the world file. **Do not** use `gz-sim-*-system` / `gz::sim::systems::*` names from Garden/Harmonic docs. Environment variable for resources is `IGN_GAZEBO_RESOURCE_PATH` (not `GZ_SIM_RESOURCE_PATH`).

## 3. Existing implementation (unverified)
- `launch/mecanum_forklift_gazebo.launch.py`: includes `ros_gz_sim` launch with `gz_args = "-r -v 3 <world>"`, converts Xacro -> URDF -> SDF itself (comment mentions adapting to the Fortress `mecanum_drive.sdf` example) and spawns with `ros_gz_sim create`; world name `mecanum_world`.
- Worlds: `empty.sdf`, `simple_warehouse.sdf`; models: shelf, mini_pallet, cargo_box, pickup/dropoff/charging station, safety_cone.
- `config/bridge_config.yaml` bridges: `/clock`, `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/fork/cmd_pos`, `/scan`, `/camera/image_raw`, `/camera/camera_info`. **No IMU, depth image/point cloud, or lift state yet.**

## 4. Design guidance
- **World design**: keep physics step/real-time factor explicit; add `Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` (render engine ogre2 for GPU lidar/camera), `Imu` system plugins in the world. Warehouse aisles sized for the robot footprint; pallet and dock models with fiducial textures when milestone 11 starts.
- **Spawning**: `ros_gz_sim create -world <w> -string <sdf>` or `-file`; pass a pose and name. For multi-robot, unique `-name` and namespaced topics.
- **URDF -> SDF**: conversion (via `ign sdf -p` / sdformat_urdf) drops or alters: `<gazebo>` extensions not understood, fixed-joint links (merged unless `preserveFixedJoint`), `mu1/mu2/fdir1` placement (must end on `<collision><surface><friction><ode>`), sensors need `<gazebo reference>` blocks. After every change inspect the generated SDF, do not assume.
- **Friction for mecanum**: see 04. Mesh colliders on wheels disabled `fdir1`-based friction in earlier trials (comment in model); keep sphere collisions unless a validated alternative exists.
- **ros2_control option** (ADR-0002): `<ros2_control>` tag with `ign_ros2_control/IgnitionSystem` hardware plugin + `<plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">` and a controller YAML. Names above are from memory of the Humble packaging and **must be verified** against the installed package (`dpkg -L`) once approved to install.
- **Multiple instances**: unique entity names, per-robot namespace for bridge topics, `ign` topics need namespacing in the SDF plugin `<topic>` tags, TF frame prefixes (see 14). Verified-by-test required; not assumed.

## 5. Testing
```bash
ign gazebo --versions
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py
ign topic -l                         # Gazebo-side topics
ros2 topic list; ros2 topic hz /scan /odom
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```
Acceptance (M4): world loads headless and with GUI; robot spawns at the requested pose and stays still with zero command; `/clock` runs; all bridged topics have publishers; real-time factor recorded.

## 6. Failure modes / troubleshooting
| Symptom | Likely cause |
|---|---|
| Plugin not loading, silent | wrong filename (Harmonic name), library not in `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` |
| Mesh not found | model path not in `IGN_GAZEBO_RESOURCE_PATH`; `package://` unresolved in SDF |
| Bridge shows no data | wrong `gz_type_name` / direction; topic name mismatch; world-scoped vs model-scoped topic |
| Sensors publish nothing | `Sensors` system missing in world, wrong render engine, no GPU/display |
| `use_sim_time` mismatches | `/clock` not bridged |

## 7. Documentation requirements
Record bridge table, world list with purpose, plugin list per model, real-time-factor measurements, GPU/headless notes in `docs/development/` and `docs/testing/`.

## 8. Verified resources
Official docs: gazebosim.org/docs/fortress; ros_gz Humble branch README. Fortress source example `mecanum_drive.sdf` (cited in the existing launch file) - not re-read by the skill author; **verify before relying on it**.
