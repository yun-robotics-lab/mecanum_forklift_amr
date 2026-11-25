# mecanum_forklift_bringup

One-command start of the simulation and of one or more robots, each in its own namespace. Structure follows the ROS2-AMR-Manipulation-Course reference (`multiple_robot_world` + `multiple_bringup`), adapted to this project. **Status: built; single robot (`robot1`) tested in simulation; two-robot isolation tests not run yet (blocked by CPU temperature on the development machine) - unverified.**

## Files
| File | What |
|---|---|
| `config/robots.yaml` | Fleet: `name`, `x_pose`, `y_pose`, `z_pose` (0.05), `yaw` per robot (default `robot1`, `robot2`) |
| `launch/robot_world.launch.py` | Gazebo + clock bridge; per robot: `robot_state_publisher`, spawn, `parameter_bridge` (spawns chained so they do not race) |
| `launch/robot_bringup.launch.py` | Waits until the last robot's `odom_ground_truth` is published, then starts each robot's stack staggered by `robot_stagger_delay`: control, lift, Nav2 + AMCL, vision, docking, mission manager, web layer |
| `launch/all.launch.py` | Includes both |
| `mecanum_forklift_bringup/fleet.py`, `sdf_tools.py` | Fleet parsing/validation, per-robot ports and bridge arguments; URDF to SDF conversion |
| `scripts/thermal_guard.sh`, `scripts/multi_tests.py` | Dev tools (not installed): stop the simulation on high CPU temperature; two-robot isolation tests |

## Run
```bash
colcon build && source install/setup.bash
ros2 launch mecanum_forklift_bringup all.launch.py                       # default fleet (2 robots)
ros2 launch mecanum_forklift_bringup all.launch.py robots:='[{"name": "robot1", "x_pose": 0.0, "y_pose": 0.0}]'   # one robot
ros2 launch mecanum_forklift_bringup all.launch.py profile:=light        # lighter sensors, AMCL, MPPI
```
**World, maps and docks.** `world:=<name>.sdf` also selects the maps: localization `maps/<name>.yaml` (AMCL) and navigation `maps/<name>_nav.yaml` (costmaps); `map:=` / `nav_map:=` (full paths) override them, and a missing map stops the launch. Docking and mission start only if the world has a dock database (`dock_database.yaml` for `simple_warehouse`, otherwise `dock_database_<name>.yaml` in the docking package). `warehouse_v2` has floor docks (`pallet_pick_1`, `pallet_drop_1..3`) and rack docks (`rack_pick_1`, `rack_drop_1..2`), not formally tested (demonstration videos only, see `docs/videos/`). When running `robot_bringup.launch.py` on its own, pass the same `world` as to `robot_world`. Docking, mission and web layer all use the database of the running world. In `warehouse_v2` the default `robot2` spawn is at the edge of a moving-cylinder path; use e.g. (-1.8, 1.6).

Useful arguments: `world` (`simple_warehouse.sdf`), `dynamic_obstacles` (false; moves the cylinders of `warehouse_v2.sdf`), `robots_file`, `robots` (inline JSON list, wins over the file), `gui` (true), `use_sim_time` (true), `robot_stagger_delay` (10 s), `controller_type` (mppi), `profile` (`full`|`light`), `enable_navigation|vision|docking|mission|web` (true), `enable_video` (false), `rviz` (false in `robot_bringup`, true in `all.launch.py`: one RViz window per robot, needs `enable_navigation`), `debug_image` (false: publish `/<robot>/aruco/image`), `address` (127.0.0.1). Separate steps: `robot_world.launch.py` then `robot_bringup.launch.py` (same fleet arguments and `profile`).

Per robot `i` (0-based): namespace `/<name>`, REST `8000+i`, rosbridge `9090+i`, video `8080+i`, TF on `/<name>/tf` and `/<name>/tf_static` (identical frame names in every robot, one tree per robot, no global `/tf`).

Try it:
```bash
ros2 topic echo /robot1/odom --once
ros2 topic pub -r 10 /robot2/cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
curl -H "Authorization: Bearer $(cat ~/.config/mecanum_forklift/web_token)" http://127.0.0.1:8001/status   # robot2
```
Dev scripts in other packages take `ROBOT_NS=/robot1`.

## Update parameters
- Add or move robots: edit `config/robots.yaml` (or pass `robots:=...`). Names must match `[A-Za-z][A-Za-z0-9_]*` and be unique; start poses must be free floor in the world.
- Stack parameters stay in each package's YAML (wildcard node keys `/**/node:` make them work in any namespace). After editing run `colcon build --packages-select <package>` and relaunch.
- Startup timing: raise `robot_stagger_delay` on a slow machine.

## Test
```bash
colcon build --packages-select mecanum_forklift_bringup
colcon test --packages-select mecanum_forklift_bringup && colcon test-result --verbose    # unit tests (fleet parsing, ports, bridge arguments, world files) + lint
```
Two-robot checks (TF isolation, command and lift cross-talk, ports, robot removal) are in `scripts/multi_tests.py` and need the fleet running:
```bash
src/mecanum_forklift_bringup/scripts/thermal_guard.sh 92 2 15 &        # stops the simulation if the CPU average reaches 92 C
python3 src/mecanum_forklift_bringup/scripts/multi_tests.py 1,2,3,4    # then case 5 (removes robot2) last
```

## Known limits
- The development machine overheats (95 C within seconds) with two robots; disabling turbo (`no_turbo`, needs root) is the planned mitigation. Real-time factor with two robots and no navigation: about 0.5.
- No traffic management between robots, tested for 2 robots only, one machine only.
- Simulation does not validate physical safety.
