# Photos and videos: shot sheet

All footage is from simulation. Raw recordings go to `~/Videos/mecanum_forklift_raw/` (outside the project); only compressed images and short GIFs go into `images/` and `gifs/`. Label sped-up clips.

## Before each scene
1. Close private windows and turn on Do Not Disturb: the recorder captures everything in the recorded area.
2. Check `tools/media/media.sh temp`. Do not start a scene while the CPU is hot, and stop if it stays near 92 C.
3. Run one launch at a time. Stop it with Ctrl-C before the next scene.
4. Arrange the windows (Gazebo left, RViz right). To record only part of the screen: `export REGION=1850x1000+70+40`.

In every terminal:
```bash
cd ~/Personal_Project/mecanum_forklift_amr && source install/setup.bash
M=tools/media/media.sh
ONE='[{"name": "robot1", "x_pose": 0.0, "y_pose": 0.0}]'
```

## Scene 1: robot and new warehouse (`warehouse_v2.sdf`, cylinders moving)
```bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py world:=warehouse_v2.sdf dynamic_obstacles:=true
```
Photos: `$M view overview && $M shot v2_overview`; repeat with `top`, `rack`, `robot`, `side`.
Video: `$M view overview && $M record v2_obstacles 30`.
Robot motion: second terminal `ros2 run mecanum_forklift_teleop keyboard_node` (strafe, turn, fork).

## Scene 2: robot in RViz (model, TF, scan, camera)
```bash
ros2 launch mecanum_forklift_description display.launch.py      # model and TF only
```
For scan and camera use the Scene 1 launch plus `rviz2 --ros-args -p use_sim_time:=true`: fixed frame `odom`, add RobotModel (`/robot_description`), TF, LaserScan (`/scan`, best effort), Image (`/camera/image_raw`).

## Scene 3: mapping
```bash
# terminal 1 (cylinders off: they stand still in the middle of their paths and will appear in this demo map)
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py world:=warehouse_v2.sdf
# terminal 2
ros2 launch mecanum_forklift_localization mapping.launch.py
# terminal 3
ros2 run mecanum_forklift_teleop keyboard_node
# terminal 4
$M record mapping_v2          # press q when the map is complete
```
This is a video demo, not the Stage B map. Do not save it into the package `maps/` folders.

## Scene 4: navigation with three controllers (`simple_warehouse.sdf`)
These commands and `$M nav` use the old world. `warehouse_v2` now has maps (`world:=warehouse_v2.sdf`), but navigation on them is not yet tested; docking and mission in `warehouse_v2` (floor docks only) are untested, so Scenes 5 and 6 use the old world. Restart the launch for each controller (`mppi`, `dwb`, `rotation_shim`):
```bash
ros2 launch mecanum_forklift_bringup all.launch.py robots:="$ONE" rviz:=true debug_image:=false \
  enable_vision:=false enable_docking:=false enable_mission:=false enable_web:=false controller_type:=mppi
# second terminal, when Nav2 is active
$M record nav_mppi            # third terminal
$M nav                        # same five goals every time
```
Write down every goal result, including failures. `rotation_shim` is untested and does not strafe. One run each is a demonstration, not a benchmark.

## Scenes 5 and 6: docking and mission (`simple_warehouse.sdf`)
```bash
ros2 launch mecanum_forklift_bringup all.launch.py robots:="$ONE" rviz:=true enable_web:=false
# docking only
ros2 action send_goal /robot1/dock_pallet mecanum_forklift_interfaces/action/DockPallet \
  "{dock_id: pallet_pick_1, navigate_to_staging: true, undock_after: true}" --feedback
# full mission (restart the launch first so the pallet is back in place)
$M mission
```
Camera views: `$M view pick`, `$M view drop`, `$M view overview`. Marker view: add an Image display on `/robot1/aruco/image` in RViz.

## After recording
```bash
$M gif ~/Videos/mecanum_forklift_raw/nav_mppi.mp4 nav_mppi 4     # 4x speed, say so in the caption
cp ~/Videos/mecanum_forklift_raw/v2_overview.png docs/media/images/
```
Watch every clip before publishing it.
