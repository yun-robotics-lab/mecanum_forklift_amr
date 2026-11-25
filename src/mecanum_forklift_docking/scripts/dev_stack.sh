#!/bin/bash
# Development helper (not installed): start / stop the simulation stack used by the docking and
# navigation trial scripts.
#   dev_stack.sh start   gazebo (headless) -> Nav2 + AMCL -> vision + docking (waits for Nav2)
#   dev_stack.sh stop    stops everything this project starts
#   dev_stack.sh stop_dock | start_dock   only the docking launch (after rebuilding it)
#   dev_stack.sh start_mission | stop_mission   the mission manager (stop = SIGINT)
#   dev_stack.sh start_web [launch args] | stop_web   REST API, e-stop, rosbridge, video
# Logs go to $LOG_DIR (default: the session scratchpad if it exists, else ~/.cache/mecanum_logs).
# Run `stop` in its own command: its process patterns would also match a shell whose command line
# contains them.
# workspace root: three levels above this script (src/<package>/scripts); override with WS=...
WS=${WS:-$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../.." && pwd)}
LOG_DIR=${LOG_DIR:-$HOME/.cache/mecanum_logs}
mkdir -p "$LOG_DIR"

stop() {
  pkill -f "ros2 launch"
  pkill -f "ign gazebo"
  pkill -f "ruby /usr/bin/ign"
  sleep 3
  for pat in controller_server planner_server smoother_server behavior_server bt_navigator \
    waypoint_follower velocity_smoother "nav2_amcl/amcl" "/map_server" lifecycle_manager \
    wheel_odometry twist_mux cmd_vel_gate lift_controller robot_state_publisher parameter_bridge \
    opennav_docking pallet_dock_workflow aruco_node pick_trials.py pick_failures.py \
    mission_manager mission_trials.py mission_failures.py web_api estop_node rosbridge_websocket \
    web_video_server web_tests.py; do
    pkill -f "$pat"
  done
  sleep 2
  echo stopped
}

start() {
  cd "$WS" || exit 1
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  nohup ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false \
    > "$LOG_DIR/sim.log" 2>&1 &
  sleep 30
  loc="$(ros2 pkg prefix mecanum_forklift_localization)/share/mecanum_forklift_localization"
  nav="$(ros2 pkg prefix mecanum_forklift_navigation)/share/mecanum_forklift_navigation"
  nohup ros2 launch mecanum_forklift_navigation single_bringup.launch.py rviz:=false \
    map:="$loc/maps/simple_warehouse.yaml" nav_map:="$nav/maps/simple_warehouse_nav.yaml" \
    > "$LOG_DIR/nav.log" 2>&1 &
  for _ in $(seq 1 30); do
    sleep 3
    st=$(timeout 3 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
    [ "$st" = "active [3]" ] && break
  done
  echo "bt_navigator: $st"
  nohup ros2 launch mecanum_forklift_vision vision.launch.py > "$LOG_DIR/vis.log" 2>&1 &
  sleep 8
  nohup ros2 launch mecanum_forklift_docking docking.launch.py > "$LOG_DIR/dock.log" 2>&1 &
  sleep 12
  echo "docking_server: $(timeout 3 ros2 lifecycle get /docking_server 2>/dev/null | head -1)"
  echo "duplicate nodes: $(ros2 node list | sort | uniq -d | tr '\n' ' ')"
}

stop_dock() {
  pkill -f "launch mecanum_forklift_docking"
  pkill -f "opennav_docking"
  pkill -f "pallet_dock_workflow"
  pkill -f "lifecycle_manager_docking"
  sleep 2
  echo stopped docking
}

stop_web() {
  pkill -INT -f "mecanum_forklift_web_bridge/web_api"
  pkill -f "web_bridge/estop_node"
  pkill -f "rosbridge_websocket"
  pkill -f "web_video_server"
  pkill -f "launch mecanum_forklift_web_bridge"
  sleep 3
  echo stopped web layer
}

start_web() {
  cd "$WS" || exit 1
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  nohup ros2 launch mecanum_forklift_web_bridge web.launch.py "$@" > "$LOG_DIR/web.log" 2>&1 &
  sleep 10
  echo "listeners: $(ss -ltn | awk 'NR>1{print $4}' | grep -E ':(8000|9090|8080)$' | tr '\n' ' ')"
}

stop_mission() {
  pkill -INT -f "mission_manager"
  sleep 3
  echo stopped mission manager
}

start_mission() {
  cd "$WS" || exit 1
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  nohup ros2 launch mecanum_forklift_task_manager mission.launch.py > "$LOG_DIR/mission.log" 2>&1 &
  sleep 6
  echo "mission nodes: $(ros2 node list | grep -c mission_manager)"
}

start_dock() {
  cd "$WS" || exit 1
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  nohup ros2 launch mecanum_forklift_docking docking.launch.py > "$LOG_DIR/dock.log" 2>&1 &
  sleep 12
  echo "docking_server: $(timeout 3 ros2 lifecycle get /docking_server 2>/dev/null | head -1)"
}

case "$1" in
  start) start ;;
  stop) stop ;;
  stop_dock) stop_dock ;;
  start_dock) start_dock ;;
  stop_web) stop_web ;;
  start_web) shift; start_web "$@" ;;
  stop_mission) stop_mission ;;
  start_mission) start_mission ;;
  *) echo "usage: $0 start|stop" ;;
esac
