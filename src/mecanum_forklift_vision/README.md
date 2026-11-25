# mecanum_forklift_vision

ArUco marker detection for pallet and station docking. Simulation-tested only (Gazebo camera, 0.10 m markers). Decision: ADR-0009. Test report: `docs/testing/2026-11-15_m11a_vision.md`.

## Nodes and files
| File | What |
|---|---|
| `mecanum_forklift_vision/pose_core.py` | ROS-free pose maths (planar approach yaw, marker corner handling) |
| `mecanum_forklift_vision/aruco_node.py` | ROS node `aruco_node`: detects markers in `camera/image_raw`, publishes poses in `target_frame` |
| `config/aruco.yaml` | Parameters (below) |
| `launch/vision.launch.py` | Starts `aruco_node` (arg `use_sim_time`, default true; `respawn=True` because of the image-stall watchdog) |
| `scripts/generate_marker_models.py` | Dev tool: regenerates the Gazebo marker models in `mecanum_forklift_gazebo/models` |

## Interfaces
| Name | Type | Direction |
|---|---|---|
| `camera/image_raw`, `camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | subscribes |
| `aruco_marker_pose` | `geometry_msgs/PoseStamped` | publishes: the target marker (`target_marker_id`), in `target_frame` |
| `aruco_dock_pose` | `geometry_msgs/PoseStamped` | publishes: target marker with planar approach yaw (used by docking) |
| `aruco/poses` | `geometry_msgs/PoseArray` | publishes: all detected markers |
| `aruco/image` | `sensor_msgs/Image` | publishes only if `publish_debug_image: true` |

A watchdog re-subscribes once if images stop for 5 s and exits (and is respawned by the launch) after 10 s. cv_bridge is not used (numpy/OpenCV clash); the node works with the system OpenCV 4.5 and a user-site OpenCV 5.0.

## Run
Needs the simulation first:
```bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false
ros2 launch mecanum_forklift_vision vision.launch.py
ros2 param set /aruco_node target_marker_id 0      # -1 = none
ros2 topic echo /aruco_dock_pose
```
`ros2 launch mecanum_forklift_vision vision.launch.py debug_image:=true` publishes `aruco/image` (or set the parameter live: `ros2 param set /aruco_node publish_debug_image true`).
Under a namespace (multi-robot) the node is `/robot1/aruco_node`; `mecanum_forklift_bringup` starts it for you.

## Update parameters (`config/aruco.yaml`)
| Parameter | Meaning |
|---|---|
| `image_topic`, `camera_info_topic` | relative, so they follow the namespace |
| `dictionary` | `DICT_4X4_50` |
| `marker_size` | side of the black square in m; must match the marker model (0.10) |
| `target_marker_id` | marker to publish; docking sets it live |
| `corner_expand_px` | **sim-only** calibration of rendered-edge blur; set `0.0` on hardware |
| `target_frame` | stable frame for published poses (`odom`) |
| `max_tf_age` | s; detections with older TF are dropped |
| `publish_debug_image` | costs ~15 MB/s, keep off unless inspecting |

Edit the YAML, then `colcon build --packages-select mecanum_forklift_vision` (or `--symlink-install`) and relaunch. Live change: `ros2 param set /aruco_node <name> <value>`.

## Test
```bash
colcon build --packages-select mecanum_forklift_vision
colcon test --packages-select mecanum_forklift_vision && colcon test-result --verbose
```
Unit tests cover the pose maths (`test/test_pose_core.py`) plus flake8/pep257. Detection accuracy in Gazebo is documented in the test report; there is no hardware test (`corner_expand_px` and `marker_size` are placeholders).
