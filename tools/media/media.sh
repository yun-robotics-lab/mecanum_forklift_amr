#!/usr/bin/env bash
# Helpers for the project photos and videos. See docs/media/README.md.
#   media.sh temp                     CPU temperature
#   media.sh view NAME                move the Gazebo camera (overview top robot side rack pick drop)
#   media.sh shot NAME                screenshot -> $RAW/NAME.png
#   media.sh record NAME [SECONDS]    screen video -> $RAW/NAME.mp4 (default: until q is pressed)
#   media.sh gif VIDEO NAME [SPEED]   short preview -> docs/media/gifs/NAME.gif
#   media.sh nav [NS]                 send the demo goal sequence (default NS /robot1)
#   media.sh mission [NS]             pick_1, drop_1, return to (0,0); v2 is untested
# REGION=WIDTHxHEIGHT+X+Y limits shot and record to that screen area (default: the whole screen).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
RAW=${RAW:-$HOME/Videos/mecanum_forklift_raw}
REGION=${REGION:-}

temp() { sensors | awk '/Package/{print "CPU " $4}'; }

geometry() {    # sets W H X Y
  local g=$REGION
  [ -n "$g" ] || g="$(xwininfo -root | awk '/Width:/{w=$2} /Height:/{h=$2} END{print w "x" h}')+0+0"
  [[ $g =~ ^([0-9]+)x([0-9]+)\+([0-9]+)\+([0-9]+)$ ]] || { echo "bad REGION '$g'" >&2; exit 2; }
  W=$(( BASH_REMATCH[1] / 2 * 2 )); H=$(( BASH_REMATCH[2] / 2 * 2 )); X=${BASH_REMATCH[3]}; Y=${BASH_REMATCH[4]}
}

countdown() { for i in 5 4 3 2 1; do echo "recording ${W}x${H}+${X}+${Y} in $i (Ctrl-C cancels)"; sleep 1; done; }

new_file() { mkdir -p "$RAW"; [ ! -e "$1" ] || { echo "$1 exists, choose another name" >&2; exit 2; }; }

camera_pose() {    # eye x y z, target x y z -> GUICamera request (roll 0)
  awk -v a="$*" 'BEGIN{split(a,v," "); dx=v[4]-v[1]; dy=v[5]-v[2]; dz=v[6]-v[3];
    yaw=atan2(dy,dx); pit=atan2(-dz,sqrt(dx*dx+dy*dy));
    cy=cos(yaw/2); sy=sin(yaw/2); cp=cos(pit/2); sp=sin(pit/2);
    printf "pose: {position: {x: %s, y: %s, z: %s}, orientation: {x: %.5f, y: %.5f, z: %.5f, w: %.5f}}\n",
      v[1],v[2],v[3], -sy*sp, cy*sp, sy*cp, cy*cp}'
}

view() {
  local p
  case $1 in
    overview) p="0 -5.5 5.0   0 0 0" ;;
    top)      p="0 -0.01 8.5  0 0 0" ;;
    robot)    p="0.9 -0.8 0.6  0 0 0.12" ;;
    side)     p="0.05 -1.2 0.35  0.05 0 0.15" ;;
    rack)     p="0.6 0.7 0.9  -0.4 2.3 0.12" ;;       # warehouse_v2
    pick)     p="0.3 -1.2 0.7  0.7 0 0.08" ;;
    drop)     p="1.6 -2.4 1.2  2.7 -1.6 0.05" ;;      # simple_warehouse drop pad
    *) echo "unknown view '$1'" >&2; exit 2 ;;
  esac
  # shellcheck disable=SC2086
  ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera \
    --reptype ignition.msgs.Boolean --timeout 3000 --req "$(camera_pose $p)"
}

goal() {    # namespace x y yaw_deg
  local qz qw
  read -r qz qw < <(awk -v d="$4" 'BEGIN{r=d*3.14159265/360; printf "%.5f %.5f\n", sin(r), cos(r)}')
  echo "goal ($2, $3, $4 deg)"
  timeout 180 ros2 action send_goal "$1/navigate_to_pose" nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $2, y: $3}, orientation: {z: $qz, w: $qw}}}}" \
    | grep -E 'Goal finished|rejected' || echo "no result within 180 s"
}

case ${1:-help} in
  temp) temp ;;
  view) view "${2:?view name}" ;;
  shot) geometry; f=$RAW/${2:?name}.png; new_file "$f"; countdown
        ffmpeg -loglevel error -f x11grab -video_size "${W}x${H}" -i "$DISPLAY+$X,$Y" -frames:v 1 "$f"; echo "$f" ;;
  record) geometry; f=$RAW/${2:?name}.mp4; new_file "$f"; temp; countdown
        echo "press q in this terminal to stop"
        ffmpeg -hide_banner -loglevel warning -stats -f x11grab -framerate 20 -video_size "${W}x${H}" \
          -i "$DISPLAY+$X,$Y" ${3:+-t "$3"} -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p "$f"
        ls -lh "$f"; temp ;;
  gif)  out=$ROOT/docs/media/gifs/${3:?gif VIDEO NAME [SPEED]}.gif
        [ ! -e "$out" ] || { echo "$out exists, choose another name" >&2; exit 2; }
        ffmpeg -loglevel error -i "${2:?video}" -filter_complex \
          "setpts=PTS/${4:-4},fps=8,scale=720:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse" "$out"
        ls -lh "$out" ;;
  nav)  ns=${2-/robot1}      # goals 1-4 and 10 of nav_trials.py (simple_warehouse.sdf)
        goal "$ns" 0.0 -1.0 0; goal "$ns" 1.2 -1.0 0; goal "$ns" 1.2 1.0 0
        goal "$ns" 0.0 1.0 180; goal "$ns" 0.0 0.0 0 ;;
  mission) ns=${2-/robot1}
        ros2 action send_goal "$ns/execute_mission" mecanum_forklift_interfaces/action/ExecuteMission \
          "{mission_id: video, mission_type: PALLET_TRANSPORT, pick_dock_id: pallet_pick_1, drop_dock_id: pallet_drop_1, return_pose: {header: {frame_id: map}, pose: {orientation: {w: 1.0}}}}" --feedback ;;
  *) sed -n '2,11p' "$0" ;;
esac
