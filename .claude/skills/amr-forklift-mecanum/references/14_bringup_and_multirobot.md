# 14 - Bringup and Multi-Robot

## 1. Purpose
Composable launch files for each stage and an approach to run several robots in one simulation without TF/topic collisions.

## 2. Launch hierarchy (target)
| Launch | Starts |
|---|---|
| `sim.launch.py` | Gazebo world + spawn + bridge + robot_state_publisher (+ drive/controllers) - exists today as `mecanum_forklift_gazebo.launch.py` (unverified) |
| `teleop.launch.py` | teleop + arbitration |
| `mapping.launch.py` | sim + SLAM Toolbox |
| `localization.launch.py` | sim + map_server + AMCL (+ EKF) |
| `navigation.launch.py` | localization + Nav2 stack |
| `docking.launch.py` | vision + docking |
| `full_system.launch.py` | navigation + lift + docking + task manager + web bridge |
Principles: each launch includes the previous; every launch takes `use_sim_time`, `namespace`, `params_file`, `world`, `headless`; no hard-coded paths; stage start-up order enforced by lifecycle managers and readiness checks rather than only `TimerAction` delays (the reference staggers with timers - fragile).

## 3. Multi-robot design (M14)
Compatibility and behaviour **must be tested**, not assumed.
- **Namespaces**: `/robot1/cmd_vel`, `/robot1/scan`, ... Nav2 supports namespacing (`nav2_bringup` `namespace` and `use_namespace` args; check the Humble `bringup_launch.py`).
- **TF isolation**: choose one: (a) per-robot frame prefixes (`robot1/base_link`) and a shared `/tf`; or (b) remap `/tf` and `/tf_static` to `<ns>/tf`, `<ns>/tf_static` per node. Nav2 params that name frames (`base_frame_id`, `odom_frame_id`, `robot_base_frame`) must match. `map` is shared. Record the decision as an ADR.
- **Gazebo**: unique model names; per-robot SDF with topic names templated by namespace (Gazebo plugin `<topic>`, `<tf_topic>`, `<child_frame_id>`, sensor topics); one bridge per robot (or one bridge config templated with `{ns}`); unique spawn poses; one `/clock`.
- **Per-robot params**: generated from templates (`RewrittenYaml` from `nav2_common` is used by `nav2_bringup`) or per-robot YAML dirs; maps shared, costmaps per robot.
- **Ports**: rosbridge / REST / video unique per robot (13).
- **Resources**: N x (Nav2 + sensors) is heavy; measure real-time factor.
- Out of scope initially: traffic management/edge reservation (reference has a traffic manager).

## 4. Tests
Single robot: full launch from clean shell -> all nodes active within TBD s, no errors in log. Two robots: both receive goals concurrently; no cross-talk (publish `cmd_vel` to robot1 -> only robot1 moves); `view_frames` per robot is a clean tree; deleting one robot does not disturb the other; real-time factor >= TBD. Acceptance (M14): documented commands start 1 and 2 robots reproducibly.

## 5. Failure modes
Global `/tf` collisions (jumping robots); one robot's AMCL publishes `map->odom` for both; bridges subscribing to the same Gazebo topic; parameter file frame names not rewritten; DDS discovery overload (consider `ROS_DOMAIN_ID` separation only if using separate processes per robot); race: Nav2 starting before TF or sim clock.

## 6. Documentation
Launch graph, argument tables, namespace/TF convention, run recipes in `docs/development/`.
