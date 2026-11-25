# Requirements (baseline draft - needs user review)

Source: project brief (2026-11-02). Items marked **TBD** need a decision.

## Functional
| ID | Requirement | Milestone |
|---|---|---|
| R1 | Robot has four mecanum wheels and a fork lift; omnidirectional motion (vx, vy, wz) | 3, 5 |
| R2 | Complete model in URDF/Xacro with Gazebo Fortress simulation | 3, 4 |
| R3 | Keyboard and PS4 teleoperation incl. fork control | 5 |
| R4 | Odometry and a documented TF tree | 6 |
| R5 | Map building (SLAM Toolbox) and map saving | 7 |
| R6 | Localization (AMCL) on saved maps | 8 |
| R7 | Autonomous navigation with Nav2, obstacle avoidance, recovery; route graph (GeoJSON) optional | 9 |
| R8 | Lift control to target height with limits and fault handling | 10 |
| R9 | Pallet/dock detection (ArUco; YOLO + depth optional) and docking | 11 |
| R10 | Mission/task execution with cancel, timeout and failure recovery | 12 |
| R11 | Web integration: REST 8000, rosbridge 9090, video streaming | 13 |
| R12 | Single-robot and multi-robot simulation bringup | 14 |
| R13 | Later: physical hardware integration | 16 |

## Constraints
Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (fixed); one workspace; Python + C++; simulation before hardware; reference repo is read-only.

## Non-functional
Safety: web/mission commands cannot bypass limits; deadman/timeouts on manual control; documented fault handling. Quality: measurable acceptance criteria, recorded test evidence.

## Open items
Robot scale and real dimensions; real lift actuator; pallet standard; sensor suite for hardware; number of robots; authentication for web; numeric tolerances (all marked TBD in roadmap).
