# Autonomous Search-and-Rescue Grid Explorer (Gazebo + PX4 + ROS 2)

A portfolio project: a ROS 2 system that connects to PX4 SITL through the
**Micro-XRCE-DDS** bridge, flies an autonomous lawnmower (grid) search pattern
over a simulated disaster zone in **Gazebo**, detects target objects with a
simulated camera (ArUco markers / color anomalies), and logs the GPS
coordinates of every finding. RViz shows live telemetry, the flown path, and
detection markers; the final showcase is a split-screen video of Gazebo + RViz.

## Documentation

| Doc | Contents |
|---|---|
| [docs/plan.md](docs/plan.md) | Master plan: goals, scope, milestones, tasks, acceptance criteria, risks |
| [docs/architecture.md](docs/architecture.md) | Technical design: components, topics, frames, algorithms, repo layout |
| [docs/research.md](docs/research.md) | Verified environment facts, install matrix, links to official docs |

## Target stack (verified 2026-08)

- Ubuntu 26.04 LTS · ROS 2 **Lyrical Luth** · Gazebo **Jetty** · PX4-Autopilot
  (main / latest stable) SITL · Micro-XRCE-DDS Agent · `px4_msgs` / `px4_ros_com`
- Fallback stack if PX4/Lyrical integration is not yet stable: Ubuntu 24.04 ·
  ROS 2 Jazzy · Gazebo Harmonic (PX4's best-tested path). See
  [docs/plan.md — Risks](docs/plan.md#risks-and-mitigations).

## Status

Planning complete — see [docs/plan.md](docs/plan.md). Implementation not started.
