# M10 lift control - test report (2026-11-14)

Setup: `ros2 launch mecanum_forklift_gazebo mecanum_forklift_gazebo.launch.py gui:=false` (includes `lift.launch.py`), headless, simple_warehouse, Humble / Fortress 6.18. Test scripts were run from a scratch directory (not stored in the repo). Simulation only; it does not validate physical safety. The real actuator, payload and feedback are unknown.

## Unit tests
`colcon test --packages-select mecanum_forklift_lift`: 15 tests (13 state-machine cases in `test_lift_core.py`: homing, wait for feedback, move, reject out-of-range/NaN/negative speed, reject when not idle, speed clamp, limits on command, cancel hold, feedback loss + reset, stall, timeout, over-limit, goals ignored in FAULT; plus flake8, pep257) - 0 failures. Combined `colcon test-result` over the six affected packages: 36 tests, 0 failures.

## Plant finding (fixed)
With the original Gazebo PID (p 1000, i 20, d 50) the fork sagged 5-9 mm under load on a fresh start and the first goal timed out (reproduced twice; settled to 1-2 mm only after the slow integrator had wound up in earlier runs). Retuned in `mecanum_forklift.gazebo.xacro` to p 5000, i 50, d 150. After that, a fresh start passed. This changes the plant; the fork-under-payload behaviour is still unknown.

## Simulation results (fresh start, retuned gains, RTF about 0.5)
| Test | Result |
|---|---|
| Start | HOMING -> IDLE at -0.001 m (the known rest offset) |
| Goals 0.06 / 0.12 / 0.0 / 0.06 m | succeeded; final error -0.98 / -0.95 / 0.00 / -0.92 mm; overshoot 0 mm; 1.5-2.7 s sim time |
| Out-of-range target 0.2, -0.01; speed -1 | aborted with code 2 (REJECTED_OUT_OF_RANGE) |
| Goal during a teleop move | aborted with code 1 (REJECTED_NOT_IDLE) |
| Cancel mid-move (at 0.031 m) | CANCELED, final height 0.0311, drift 0.00 mm over 2 s, state IDLE |
| Teleop target +5 / -5 m | fork stops at 0.1192 / -0.000 m; never above 0.1192 or below -0.001 |
| `lift_state` rate | 10 Hz of sim time (7.4-7.8 Hz wall at RTF 0.5) |

## Fault injection (isolated ROS domain, fake `/joint_states` plant, real `lift_controller`)
| Test | Result |
|---|---|
| Feedback stops mid-move | FAULT_NO_FEEDBACK, action aborted code 4 after 1.11 s (timeout 1.0 s); `feedback_available` false |
| Goal while FAULT | aborted code 1 |
| Reset without feedback | refused; with feedback accepted, state IDLE (holding) |
| Height frozen during move (stall) | FAULT_STALL, action aborted code 5 after 1.91 s (stall_time 1.5 s); reset accepted |
| Timeout, over-limit | unit tests only |

## Acceptance (approved criteria)
- Height error <= 3 mm at 0, 0.06, 0.12 m: met in sim (<= 1 mm).
- No travel outside 0..0.12 m: met in sim (min -0.001 m = rest offset).
- Out-of-range goal rejected: met. Cancel holds within 5 mm: met (0 mm).
- Feedback loss -> FAULT within 1 s of the timeout: met (1.11 s from loss, 0.11 s after the 1.0 s timeout).
- Stall: met with a fake plant; not reproduced with Gazebo.
- State at about 10 Hz: met in sim time.

## Known issues / unverified
- `/joint_states` from Gazebo runs at about 700 Hz; the controller copes, but it is wasteful (not changed here).
- Not tested: real PS4/keyboard through `fork/cmd_teleop` (topic rename only, built, lint clean), fork under payload, over-limit/timeout in Gazebo, driving interlock (milestone 12), hardware.
- Timeout/stall values are sim placeholders.
- The tests above ran after restarts that left orphan processes; the passing run was done after cleaning them.
