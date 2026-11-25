# M11b - pallet engagement and undock end pose (simulation) - 2026-11-19

Follow-up to docs/testing/2026-11-19_m11b_final_alignment.md, which left two open points: the pallet tilted after the lift
and the undock ended far from the staging pose.

## Pallet engagement
Geometry (fork.stl, mini_pallet model): two tines 25 mm wide at +-62.5 mm, 6 mm thick, reaching 0.150 m ahead of `fork_link`
(tip = base_link + 0.368 m); fork carriage front = fork_link + 0.010 m; pallet 0.26 x 0.20 m, pockets 85 mm wide at +-57.5 mm
(lateral margin ~35 mm), pallet centre of mass 0.10 m from its end face, deck underside 0.04 m.

| stop distance (base_link before marker) | tine insertion | pallet after lift | pallet carried during undock |
|---|---|---|---|
| 0.278 m (previous) | 0.093 m (tip at the centre of mass) | tilted 10 -> 14 deg (18 deg once settled), z 0.034 m | yes, tilted |
| **0.245 m** (now) | **0.133 m** | **flat, tilt 0.00 deg**, z 0.034 m | yes, flat |

Cause (supported by the trace: at the end the pallet centre sat exactly on the tine tips): with 0.093 m insertion the pallet's
centre of mass was at the tine tip, so it tipped over the tip edge when lifted. 0.245 m is also close to the limit: carriage front
at fork_link + 0.010 = 7 mm short of the pallet end face; the longitudinal tolerance is +-10 mm (internal 5 mm), so do not
reduce the distance further without changing the margin. Marker boards have no collision, so they never block the carriage.

## Undock end pose (explained, not a bug)
opennav_docking backs up by `staging_x_offset` (0.25 m) from the pose where the dock ended and stops within
`undock_linear_tolerance` (0.05 m): docked x 0.526 -> ended 0.326; docked x 0.559 -> ended 0.358 (both = docked - 0.25 + 0.05).
It does not return to the absolute staging pose. Backing out takes ~25 s wall (~12 s sim) because the speed decays toward the end.

## Re-run with stop distance 0.245 m (config changes: `stop_distance` 0.245, `external_detection_translation_x` -0.395, dock pose x 0.415)
- Trials, N = 21 (pallet lateral 0 / +-5 cm x yaw 0 / +-10 deg, 3 repeats): **21/21** within 10 mm / 10 mm / 2 deg;
  longitudinal -5.5 to -6.4 mm, lateral <= 3.3 mm, yaw <= 0.67 deg, 16.6-22.8 s per dock; pallet z 34.5-34.9 mm, **tilt 0.00 deg in every trial**.
- Failure injection 7/7 (unknown dock, marker not visible, cancel, total timeout, lift above limit, second goal rejected, dock + lift + undock);
  robot stopped after each.
- Package tests: docking 42 passed (lint + `dock_core` + `align_core`).

## Not verified
Pallet weight/payload realism (sim masses 0.3 kg + 0.2 kg box), other pallet types, offsets beyond +-5 cm / +-10 deg, driving with the load,
Nav2 staging (M9), hardware. Simulation does not establish physical safety.
