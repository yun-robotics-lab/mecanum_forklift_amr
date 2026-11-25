# mecanum_forklift_lift

Fork lift controller: state machine, `SetLiftHeight` action server, limits, fault handling. Simulation only (it drives the Gazebo fork position controller). Decision: ADR-0008. Test report: `docs/testing/2026-11-14_m10_lift_control.md`.

## Nodes and files
| File | What |
|---|---|
| `mecanum_forklift_lift/lift_core.py` | ROS-free state machine (`LiftCore`): ramp, settle, timeout, stall, feedback loss, over-limit |
| `mecanum_forklift_lift/lift_controller.py` | ROS node around the core |
| `config/lift.yaml` | All limits and timeouts (heights m, speeds m/s, times s of node/sim time) |
| `launch/lift.launch.py` | Starts `lift_controller` (arg `use_sim_time`, default true); also included by the Gazebo launch |

## Interfaces
| Name | Type | Direction |
|---|---|---|
| `set_lift_height` | action `mecanum_forklift_interfaces/SetLiftHeight` | server |
| `lift_state` | `mecanum_forklift_interfaces/LiftState`, 10 Hz | publishes |
| `lift_reset_fault` | `std_srvs/Trigger` | server (needs fresh feedback) |
| `joint_states` | `sensor_msgs/JointState` (joint `fork_joint`) | subscribes (feedback) |
| `fork/cmd_teleop` | `std_msgs/Float64` target height | subscribes (teleop; clamped; ignored during an action or FAULT) |
| `fork/cmd_pos` | `std_msgs/Float64` | publishes (to Gazebo via the bridge) |

States: UNINITIALIZED -> HOMING -> IDLE <-> MOVING -> FAULT (reset -> IDLE, holding height).
Rejected goals are accepted and aborted at once with a code (1 not idle, 2 out of range) so clients get a reason.

## Run
Normal use: start the simulation (the lift controller is included).
```bash
ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false
```
Standalone: `ros2 launch mecanum_forklift_lift lift.launch.py`

Try it:
```bash
ros2 topic echo /lift_state
ros2 action send_goal /set_lift_height mecanum_forklift_interfaces/action/SetLiftHeight "{target_height: 0.06, speed: 0.0}" --feedback
ros2 topic pub --once /fork/cmd_teleop std_msgs/msg/Float64 "{data: 0.1}"
ros2 service call /lift_reset_fault std_srvs/srv/Trigger
```

## Test
```bash
colcon build --packages-select mecanum_forklift_interfaces mecanum_forklift_lift
colcon test --packages-select mecanum_forklift_lift && colcon test-result --verbose
```
Unit tests (`test/test_lift_core.py`) cover the state machine with a simulated plant; flake8/pep257 are included. Simulation checks (moves, cancel, rejection, teleop clamp) and fault injection with a fake `/joint_states` are described in the test report; those scripts were run from a scratch directory and are not stored in the repo.

## Developing
- Put logic in `lift_core.py` (no ROS imports) and add a unit test first; keep the node thin.
- Heights must stay inside the URDF joint limits (`fork_lift_max` = 0.12 m).
- The Gazebo fork PID (5000/50/150) lives in `mecanum_forklift_description/urdf/mecanum_forklift.gazebo.xacro`; weak gains make the fork sag and time out.
- Known gaps: no payload test, no interlock with driving (milestone 12), no hardware backend; timeouts are sim placeholders. Simulation does not validate physical safety.

## Update parameters
All limits and timeouts are in `config/lift.yaml` (node key `/**/lift_controller`); change, rebuild (`colcon build --packages-select mecanum_forklift_lift`) and relaunch, or live with `ros2 param set /lift_controller <name> <value>`. Heights must stay within `fork_lift_max` (0.12 m).
