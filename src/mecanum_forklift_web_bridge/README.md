# mecanum_forklift_web_bridge

Web layer for the forklift: REST API, software e-stop, rosbridge and video launch, minimal dashboard. **Status: tested in simulation 2026-11-24, not verified** (`docs/testing/2026-11-24_m13_web.md`, ADR-0013). It never publishes velocity or lift commands; missions go through `ExecuteMission`.

## Requirements
`fastapi` and `uvicorn` as pip packages (tested: fastapi 0.116.1, uvicorn 0.35.0; the Ubuntu apt versions are too old), `rosbridge_server` and `web_video_server` (apt, ROS Humble). Do not import `cv2`/`numpy` in this package (the user-site numpy 2 / OpenCV 5 clash with ROS).

## Ports and binding
| Service | Port | Process |
|---|---|---|
| REST API + dashboard (`/`) | 8000 | `web_api` |
| rosbridge WebSocket | 9090 | `rosbridge_websocket` (topics `/mission_status`, `/lift_state`, `/estop/state` only) |
| MJPEG video | 8080 | `web_video_server` |
All on `127.0.0.1`; `ros2 launch mecanum_forklift_web_bridge web.launch.py address:=<ip> rest_port:=8001 rosbridge_port:=9091 video_port:=8081 enable_video:=false`. Check with `ss -ltn`.
Only open `address` on a trusted network: rosbridge 2.0.4 has no authentication.

## REST API (all but /health and `/` need `Authorization: Bearer <token>`; token file `~/.config/mecanum_forklift/web_token`)
| Method, path | Purpose |
|---|---|
| `GET /health` | `{status, ros}` (no token) |
| `GET /status` | pose (map), lift, mission, e-stop, mission manager availability, video URL; battery/diagnostics `null` |
| `GET /docks` | dock database entries a mission may use (name -> operation, type) |
| `GET /events` | Server-Sent Events, the status JSON at 2 Hz |
| `POST /missions` | `{pick_dock_id, drop_dock_id, mission_id?, mission_type?, return_pose?{x,y,yaw?}}` -> 202 `{mission_id, state: SUBMITTED}` |
| `GET /missions/{id}` | state, step, success, error_code, failed_step |
| `POST /missions/{id}/cancel` | cancel an active mission |
| `POST /estop` | engage the software e-stop (never rate limited) |
| `POST /estop/release` | always 403 `LOCAL_ONLY`: release on the robot with `ros2 service call /estop/release std_srvs/srv/Trigger` |
Errors: `{error_code, message, details}` with 400 INVALID_JSON / INVALID_PAYLOAD / INVALID_MISSION_ID / UNKNOWN_DOCK / WRONG_DOCK_OPERATION / POSE_OUT_OF_BOUNDS, 401 UNAUTHORIZED, 403 LOCAL_ONLY, 404 NOT_FOUND, 409 BUSY / DUPLICATE_MISSION_ID / ROBOT_FAULT / NOT_ACTIVE, 413 PAYLOAD_TOO_LARGE, 423 ESTOP_ENGAGED, 429 RATE_LIMITED, 503 ROS_UNAVAILABLE, 500 INTERNAL_ERROR. OpenAPI at `/docs`.

```bash
T=$(cat ~/.config/mecanum_forklift/web_token)
curl -H "Authorization: Bearer $T" -H 'Content-Type: application/json' -d '{"pick_dock_id":"pallet_pick_1","drop_dock_id":"pallet_drop_1"}' http://127.0.0.1:8000/missions
```
Run: `src/mecanum_forklift_docking/scripts/dev_stack.sh start && ... start_mission && ... start_web`; tests: `python3 src/mecanum_forklift_web_bridge/scripts/web_tests.py`.

## Update parameters
`config/web.yaml` (`web_backend`, `estop_node`): REST rate limit, payload limit, token file, goal-pose bounds, e-stop behaviour. Launch arguments (ports, `address`, `enable_video`, `topic_ns`) are listed above. Rebuild with `colcon build --packages-select mecanum_forklift_web_bridge` and relaunch. Rotate the token by deleting `~/.config/mecanum_forklift/web_token` (a new one is created at start-up).

## Test
```bash
colcon build --packages-select mecanum_forklift_web_bridge
colcon test --packages-select mecanum_forklift_web_bridge && colcon test-result --verbose
python3 src/mecanum_forklift_web_bridge/scripts/web_tests.py      # needs the running stack
```
Unit tests cover validators, auth, API and the rate limit with a fake backend.

## Multi-robot
Run one instance per robot with its own ports (bringup uses REST 8000+i, rosbridge 9090+i, video 8080+i) and `topic_ns:=/robotN`, which scopes the rosbridge topic list and the video URL. rosbridge has no authentication: keep it on a trusted network.
