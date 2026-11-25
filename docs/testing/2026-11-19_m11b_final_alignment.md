# M11b - DockPallet with holonomic final alignment (simulation) - 2026-11-19

> Note: these runs used stop distance 0.278 m. The later stop distance 0.245 m (pallet no longer tilts) and the explanation of the
> undock end pose are in docs/testing/2026-11-19_m11b_engagement_undock.md, which repeats the 21-trial and failure runs.

Follows docs/testing/2026-11-15_m11b_docking.md (stock opennav controller: 1/7). Change: opennav ends its approach
0.43 m before the marker (`docking_threshold` 0.05, `external_detection_translation_x` -0.43); the workflow then runs
`final_align` (vx, vy, wz, `align_core.py`) on the marker pose latched while the marker was >= 0.5 m away, until
|long| <= 5 mm, |lat| <= 5 mm, |yaw| <= 1 deg for 0.5 s (half of the acceptance tolerance, as margin for the marker
pose error), then lifts. Velocity: `cmd_vel_dock` -> `twist_mux` -> `cmd_vel_gate`; zero velocity is sent on every exit.

## Automated checks
- Docking package: 42 passed (lint, `dock_core`, `align_core`: kinematic simulation converges to tolerance from lateral
  0/+-5/8 cm x yaw 0/+-10/15 deg x 0.05/0.15 m short, overshoot < 2 cm, limits respected, forward speed waits for alignment).
  Vision: 8 passed. Build ok.

## Trials (start (0,0,0); pallet + box + marker moved rigidly by the offset; truth = marker-derived target)
Success = action success and |long| <= 10 mm, |lat| <= 10 mm, |yaw| <= 2 deg (acceptance tolerance).

| run | internal tolerance | result |
|---|---|---|
| 1 x 7 | 10 mm / 10 mm / 2 deg | 0/7: converged, but stopped at the band edge (long -10.1 to -10.9 mm) |
| 1 x 7 | 5 mm / 5 mm / 1 deg | 7/7 |
| 3 x 7 (N = 21) | 5 mm / 5 mm / 1 deg | **21/21** |

N = 21 final run: cases pallet lateral 0 / +-5 cm and yaw 0 / +-10 deg (7 combinations x 3). Longitudinal error -5.5 to -6.9 mm
(always a few mm short, systematic), lateral <= 3.7 mm, yaw <= 0.8 deg; 16-26 s per dock at RTF ~0.5.
Acceptance (>= 90 % within tolerance, simulation): **met** for these offsets.

## Failure injection (7 cases, all pass; robot ground-truth displacement < 2 mm over 2 s after each)
| case | expected | got |
|---|---|---|
| unknown dock id | DOCK_NOT_IN_DB | DOCK_NOT_IN_DB (robot did not move) |
| marker not visible (moved away) | MARKER_NOT_FOUND | MARKER_NOT_FOUND |
| cancel 7 s into the dock | CANCELED | CANCELED |
| total timeout 6 s | TIMEOUT | TIMEOUT |
| lift target above the 0.12 m limit | LIFT_FAILED | LIFT_FAILED |
| second goal while docking | rejected | rejected; first goal SUCCESS |
| dock + lift + undock | SUCCESS | SUCCESS |

## Observations / not verified
- Undock reports success but the robot ended at x = 0.325 m, not at the staging pose (0.13 m); cause not investigated.
  The forks are clear of the pallet (tip 0.69 m < pallet end 0.8 m), whether the pallet came with them was not checked.
- Pallet engagement quality (fork/pocket fit, the pallet tilted ~14 deg after the first lift in the earlier run) not investigated.
- Marker loss during the approach is handled by design (latched pose), not as a failure; only "not visible at start" fails.
- Not tested: Nav2 staging (needs M9), pallets rotated more than +-10 deg or offset more than 5 cm, start poses other than 0.13 m
  from staging, hardware, payload, driving interlock. Simulation does not establish physical safety.
- Scripts: `scripts/dock_trials.py`, `scripts/dock_failures.py` (need the sim, vision and docking launches).
