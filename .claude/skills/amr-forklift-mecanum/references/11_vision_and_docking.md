# 11 - Vision and Docking

## 1. Purpose
Detect markers/pallets, estimate poses, maintain a dock database, and execute docking with measurable alignment and failure recovery. Mecanum lateral motion can simplify final alignment but also needs tighter control of yaw.

## 2. Compatibility (verified 2026-11-02)
| Item | Status |
|---|---|
| `cv_bridge` 3.2.1, OpenCV via ROS | installed |
| ArUco ROS package (`aruco_opencv`, `ros2_aruco`) | **not installed**; reference repo has its own `forklift_aruco` node using OpenCV (`calib.yaml`/`calib.npz` at repo root). Choice open (Q-V1) |
| `opennav_docking` | apt candidate 0.0.2, **not installed**; reference docking uses it with 4 dock types. Need to review the Humble-backport API before reuse. Alternative: a custom docking action server |
| `vision_msgs` | in ROS share; `depth_image_proc` not installed; `realsense2_camera` present |
| YOLO / PyTorch | not checked; GPU not checked; requires explicit approval before installing |
Camera currently in the model: `front_camera_link` on the fork carriage publishing `/camera/image_raw` and `/camera/camera_info` (RGB only; no depth bridged).

## 3. Components
1. **Marker detection**: OpenCV `cv2.aruco` (dictionary and marker size fixed in config); outputs marker pose in camera optical frame; requires camera calibration (`camera_info`; in sim take values from the Gazebo sensor's FOV/resolution).
2. **Pallet detection**: options (a) fiducial on pallet, (b) depth camera + plane/pocket fitting, (c) YOLO on RGB-D. Start with (a) in simulation; add (b)/(c) only with approval and real data needs.
3. **Transforms**: the camera is mounted on the lift carriage, so `base_link -> front_camera_link` varies with fork height. Use TF with the live `fork_joint` state; do not hard-code the camera pose. Always transform detections to a stable frame (`odom`/`map`) with timestamp and handle detection latency.
4. **Dock database** (YAML): `dock_id`, type (charge / station / pallet_pick / pallet_drop), pose in map, marker id, approach offset, lift height, tolerances, timeouts. Reference uses `dock_database.yaml` (referenced in `ExecuteTask`).
5. **Docking action**: `DockRobot`/`UndockRobot` (or reuse reference types) - goal: dock_id; result: success + error code; feedback: state, errors.

## 4. Docking state machine (proposal)
`IDLE -> APPROACH_STAGING (Nav2) -> ACQUIRE_MARKER -> ALIGN (vx, vy, wz control) -> FINAL_APPROACH -> ENGAGE (lift / contact) -> VERIFY -> DONE`; failure edges to `RETRY` (limited count: back off, re-acquire) and `ABORT` (stop, report). Cancel from any state stops motion.
**Alignment criteria must be numeric and approved before implementation.** Placeholder starting values (not validated): lateral error <= 10 mm at 0.3 m model scale, heading error <= 2 deg, distance error <= 10 mm, marker lost > 1 s -> hold then retry, total timeout 60 s, max 2 retries.
Mecanum advantage: control vy for lateral error and wz for heading independently; use a controller (PID or pure proportional with deadband) with velocity limits lower than navigation limits.

## 5. Tests (simulation)
Place the pallet/station with offsets (lateral +-5 cm, yaw +-10 deg, distance 0.5-1.5 m); run N trials each; record success rate, final errors, time; test marker occlusion, wrong marker id, pallet missing, timeout, cancel mid-dock, lift failure during engage. Detection unit test with recorded images / sim ground truth for pose error.
Acceptance (M11): success >= TBD % over N runs with errors within the approved tolerance; every failure path ends in safe stop with a distinct error code.

## 6. Failure modes
Wrong optical-frame convention (camera x-right,y-down,z-forward); marker ambiguity at oblique angles; motion blur or exposure in sim vs real; camera pose wrong because fork height ignored; stale detections used for control; docking oscillation from latency; Nav2 staging pose placed where costmap forbids.

## 7. Documentation
Dock database schema and entries, marker/dictionary/size table, calibration source, state diagram, tolerance table, trial results.

## 8. Open
Camera/depth hardware, pallet standard (size, pockets, marker placement), real tolerances, safety interlocks. **Simulation success does not imply physical safety.**
