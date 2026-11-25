# 05 - Teleoperation

## 1. Purpose
Manual control (keyboard, PS4) and how manual commands coexist with autonomous commands.

## 2. Current state (inspected 2026-11-02, unverified)
Package `mecanum_forklift_teleop` (ament_python; deps rclpy, geometry_msgs, std_msgs, sensor_msgs, joy):
- `keyboard_node.py`: publishes `geometry_msgs/Twist` on `cmd_vel` (vx, vy, wz) at `publish_rate` 20 Hz and `std_msgs/Float64` on `fork/cmd_pos`; params `linear_speed` 0.2, `angular_speed` 0.8, `fork_step` 0.01, `fork_min` 0.0, `fork_max` 0.12. It publishes a zero `Twist` on shutdown.
- `ps4_node.py` + `config/ps4.yaml` + `launch/ps4_teleop.launch.py`: subscribes to `joy`; axes vx=1, vy=0, wz=3; **L1 (button 4) = enable**, R1 (5) = turbo (x2.0), Triangle/Cross/Square = fork up/down/reset; `linear_speed` 0.3, `angular_speed` 1.0.
Whether the PS4 node publishes zero on enable release, handles a stale `joy`, and clamps fork limits was **not verified** - inspect before claiming deadman behaviour. PS4 axis/button indices differ between drivers (hid-playstation vs hid-sony vs ds4drv); confirm with `ros2 topic echo /joy`.

### Update 2026-11-06 (tested, see docs/testing/2026-11-06_m5_teleop_safety.md)
Both nodes publish to `cmd_vel_teleop`; `cmd_vel_gate` (0.5 s watchdog, limits 0.6/0.6/2.0) is the only `cmd_vel` publisher. PS4: release publishes zero, stale `/joy` stops after `joy_timeout`, fork clamped to [0, 0.12] - all tested with fake Joy. Keyboard tested through a pty. Ctrl-C shutdown bug (invalid context) fixed. Real PS4 indices remain unverified.

Key/button map: keyboard w/x = +/-vx, a/d = +/-vy, q/e = +/-wz, space/k/s stop, r/f/v fork up/down/bottom, t/g and y/h = linear/angular speed +/-10 %. PS4: hold L1 to drive; left stick vx/vy, right stick x wz, R1 turbo x2, D-pad up/down speed scale, Triangle/Cross fork up/down, Square fork bottom.

## 3. Design
- Teleop publishes to `cmd_vel_teleop` (not directly `cmd_vel`) once arbitration exists. Today it writes `cmd_vel` directly - acceptable only before Nav2 is added (M9).
- Arbitration: `twist_mux` (installed, 4.3.0) with priorities, e.g. emergency stop > teleop > dock > navigation, per-source timeouts, and a lock topic. A velocity smoother (Nav2 `nav2_velocity_smoother`, installed) and collision monitor (`nav2_collision_monitor`, installed) sit between the mux and the drive. Chain order and topic names: decide in M9 and record an ADR.
- Alternative: `teleop_twist_joy` / `teleop_twist_keyboard` (installed) - generic, but have no fork support; custom nodes kept for fork control.

## 4. Safety behaviour (requirements to verify)
- Deadman: no motion unless enable held; releasing publishes zero.
- Timeout: if no input for T (e.g. 0.5 s, TBD), command goes to zero (mux per-source timeout, and controller `reference_timeout`).
- Limits: speed caps in YAML; turbo explicitly bounded.
- Fork commands bounded by lift limits (10), not just teleop limits.
- Web/mission commands must not be able to override teleop deadman/e-stop (13).

## 5. Tests
- Unit: key -> twist mapping; joy -> twist with deadzone, enable, turbo.
- Sim: publish synthetic `/joy` messages (`ros2 topic pub`) and check `/cmd_vel`; kill the teleop node and confirm robot stops within the timeout; verify strafe via the keyboard keys.
Acceptance: each axis moves the robot in the documented direction; release/timeout stops within TBD s; fork clamps to [0, fork_max].

## 6. Failure modes
Robot keeps moving after controller disconnect (no timeout); axis indices wrong; dual publishers on `cmd_vel` fighting; terminal raw-mode left broken after crash (keyboard node).

## 7. Documentation requirements
Key/button map table, parameter table, arbitration priority table.
