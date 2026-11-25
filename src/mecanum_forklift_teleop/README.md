# mecanum_forklift_teleop

Keyboard and PS4 teleoperation. Drive commands go to `cmd_vel_teleop` (then twist_mux -> `cmd_vel_gate`); fork targets go to `fork/cmd_teleop` (the lift controller clamps them). Nothing here publishes `cmd_vel` or `fork/cmd_pos` directly.

## Nodes
| Node | Input | Output |
|---|---|---|
| `keyboard_node` | terminal keys | `cmd_vel_teleop` (Twist, 20 Hz, latched direction), `fork/cmd_teleop` (Float64) |
| `ps4_node` | `/joy` (from `joy_node`) | `cmd_vel_teleop` only while L1 is held (deadman), `fork/cmd_teleop` while a fork button is held |

Keyboard: w/x forward/back, a/d strafe, q/e rotate, space or k stop, r/f fork up/down, v fork to bottom, t/g and y/h change speeds.
PS4: hold L1 to drive; R1 turbo; Triangle/Cross fork up/down; Square fork to bottom. `joy_timeout` 0.3 s without `/joy` -> zero velocity.

## Files
`config/ps4.yaml` (speeds, deadzone, axis/button indices, fork rate and limits), `launch/ps4_teleop.launch.py` (starts `joy_node` and `ps4_node`; arg `device_id`, default 0).

## Run
Needs the simulation running first (`ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py`).
```bash
ros2 run mecanum_forklift_teleop keyboard_node        # needs a real terminal (raw keys)
ros2 launch mecanum_forklift_teleop ps4_teleop.launch.py device_id:=0
```
Check button/axis numbers of your pad with `ros2 topic echo /joy` and edit `ps4.yaml`.

## Test
```bash
colcon build --packages-select mecanum_forklift_teleop
colcon test --packages-select mecanum_forklift_teleop
```
No unit tests exist. Behaviour was checked in simulation with a real PS4 and keyboard before the lift controller existed (`docs/testing/2026-11-06_m5_teleop_safety.md`); the fork path via `fork/cmd_teleop` was changed afterwards and is **unverified** with real devices. A fake `/joy` publisher is the easiest way to test without a pad.

## Developing
- Keep the deadman behaviour: no command while L1 is released, zero on joy loss.
- New command sources must go through arbitration (`cmd_vel_teleop`-style topic + `twist_mux.yaml`), not straight to `cmd_vel`.

## Update parameters
Controller mapping, speeds and deadzones: `config/ps4.yaml`; keyboard parameters `linear_speed` (0.2 m/s), `angular_speed` (0.8 rad/s), `fork_step` (0.01 m), `fork_min`, `fork_max` (`ros2 run mecanum_forklift_teleop keyboard_node --ros-args -p linear_speed:=0.3`). Rebuild with `colcon build --packages-select mecanum_forklift_teleop`. For a namespaced robot remap the output: `-r cmd_vel_teleop:=/robot1/cmd_vel_teleop`.
