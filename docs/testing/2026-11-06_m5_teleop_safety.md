# 2026-11-06 - Milestone 5, teleop and stop-latency tests

Setup: headless `mecanum_forklift_gazebo.launch.py gui:=false` (includes control launch: `cmd_vel_gate`, `wheel_odometry`), `ps4_node` with `config/ps4.yaml`, RTF ~0.6. Fake `/joy` and a pseudo-terminal for the keyboard node; no real PS4 or keyboard hardware. Scripts were run from a scratch dir.

## Approved tolerances (user accepted 2026-11-06)
Displacement error <= 2 %, yaw error <= 1 deg, stop <= 1.0 s (wall clock) after release, joy loss or publisher loss. Wall clock is used because the gate watchdog is wall clock; sim-time values are listed for reference.

## Results
| Check | Result | Status |
|---|---|---|
| PS4 deadman: stick forward without L1 | 0 non-zero commands | tested (fake Joy) |
| PS4 mapping | fwd vx 0.30; axis0 +1 -> vy +0.30 (left); axis3 +1 -> wz 1.00; turbo vx 0.60 | tested (fake Joy) |
| PS4 release L1 | zero published after 0.005 s, no non-zero afterwards | tested |
| PS4 /joy lost while enabled | zero after 0.534 s (joy_timeout 0.5) | tested |
| PS4 fork clamp | max 0.1200, min 0.0000 | tested |
| Keyboard mapping (pty) | w/x/a/d/q/e give +vx/-vx/+vy/-vy/+wz/-wz at 0.2 m/s, 0.8 rad/s; t raises speed 10 %; space stops | tested (pty, not a physical keyboard) |
| Keyboard fork clamp | 15x r -> 0.12; 20x f -> 0.0 | tested |
| Stop on release (ground truth speed < 0.01 m/s, from 0.30 m/s) | 0.33, 0.33, 0.33, 0.41 s wall (0.24-0.26 s sim) | tested, meets <= 1.0 s |
| Stop on /joy loss | 0.81, 0.85 s wall (0.64 s sim) | tested, meets <= 1.0 s, margin only ~0.15 s |
| Stop on teleop publisher loss (gate timeout) | 0.80, 0.92 s wall (0.56-0.60 s sim) | tested, meets <= 1.0 s, margin only ~0.08 s |
| `colcon test` control + teleop | 16 tests, 0 failures (teleop has lint only) | tested |
| Strafe/rotate/diagonal regression | see 2026-11-06_m3-4_cleanup_step_b.md (<= 1 mm cross-axis, 0.0 deg yaw) | tested |

## Bug found and fixed
Ctrl-C / SIGINT on `keyboard_node` and `ps4_node`: rclpy's own signal handler invalidated the context, so the shutdown zero-command `publish` raised `RCLError` ("publisher's context is invalid"). In `keyboard_node` this also happened before `termios.tcsetattr`, leaving the terminal in cbreak mode, and no zero was sent. Fix: `SignalHandlerOptions.NO`, restore the terminal first, publish the zero and wait 0.1 s before destroying the node. After the fix: keyboard last command after Ctrl-C = (0, 0, 0); PS4 last vx after SIGINT = 0.0.

## Not verified
- Real PS4 axis/button indices (no `/dev/input/js*` here).
- Physical keyboard feel and a real terminal restore (only a pty was used).
- Stop times on real hardware; speed limits are sim placeholders. Simulation does not validate physical safety.
- Stop margin on joy loss / publisher loss is small at RTF 0.6 (wall-clock timers: 0.5 s timeout + deceleration). Raising RTF or lowering `timeout` / `joy_timeout` would widen it; no change made.
- Test run hit a mid-run confound: two of the first stop runs started with the robot blocked or slowed (0.125 and 0.000 m/s before release); they were discarded and repeated with direction alternated and speed checked.

## Addendum: real PS4 (DualShock 4 over Bluetooth), 2026-11-06
- First attempt failed because bluetoothd rejected the HID connection from a "!bonded" device (stale pairing); no `/dev/input/js*` existed. Re-pairing by the user fixed it (`/dev/input/js0`, `joy_node` 8 axes / 13 buttons).
- 60 s recording while the user drove (`/joy` 1043 msgs): buttons seen 4 (L1), 5 (R1), 6; axis 1 reached +1.0 (stick forward), axis 3 reached -1.0 (right stick); `/cmd_vel_teleop` 38 msgs (34 non-zero), `/cmd_vel` non-zero seen. User reports the robot drives in the Gazebo GUI via the PS4. Fork buttons also work.
- Not captured in the recording: axis 0 (strafe) never left 0.0, so the strafe axis index on real hardware is confirmed only by the user's report, not by recorded data. Direction/sign of each axis against robot motion was not measured on real input.

## Addendum 2: timeouts lowered to 0.3 s (gate `timeout`, PS4 `joy_timeout`), 2026-11-06
Parameters read back from the running nodes: 0.3 / 0.3. Publisher-loss stop (from 0.30 m/s, ground-truth speed < 0.01 m/s), 4 runs: 0.86, 0.86, 0.89, 0.83 s wall; 0.38-0.42 s sim (before: 0.80-0.92 s wall, 0.56-0.60 s sim). All meet <= 1.0 s wall. The wall-clock gain is small because the measured time is dominated by the 2 m/s^2 deceleration stretched by RTF ~0.5 (plus 20 Hz publish and the speed-window detection); in sim time the improvement is clear. The `joy_timeout` 0.3 case and `/joy` loss were not re-measured. User-run checks in a real terminal: keyboard node and PS4 control reported working (no measurements). User-visible log: `cmd_vel_gate` warns "command timeout: publishing zero velocity" every time input stops; expected.
