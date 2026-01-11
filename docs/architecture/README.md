# Architecture

System design and development patterns for the robotlab platform.

## Architecture Layers

```
+-------------------+
| 5. Executor       |  Trust boundary: schemas + safety constraints
+-------------------+
| 4. LLM/VLM        |  Scene analysis + goal interpretation + planning
+-------------------+
| 3. Scene Graph    |  Persistent world model from perception
+-------------------+
| 2. Skills         |  ROS 2 actions/services (NavigateToPose, Pick, Place)
+-------------------+
| 1. Robot I/O      |  Sim or hardware drivers (standard ROS 2 topics)
+-------------------+
```

**Key insight:** Layer 1 (Robot I/O) is the only layer you swap when changing robots. Everything else stays the same.

---

## Non-Negotiable Principles

| Principle | Rationale |
|-----------|-----------|
| **ROS 2 is the thin waist** | Every robot presents the same contract (sensors in, commands out, common TF tree) |
| **LLM outputs are untrusted** | LLMs propose; executor validates schemas + enforces safety; LLMs never command raw motors |
| **Drones require autopilot** | PX4 or ArduPilot owns stabilization/safety; ROS 2 runs "above" it via offboard control |
| **Record everything** | Every demo must be rosbag2-loggable for replay debugging |
| **Sim time from Day 1** | Always use `/clock` + `use_sim_time:=true` for nav2, TF, EKF, planners, executive |

---

## Documentation Index

| Document | Purpose |
|----------|---------|
| [Topic Conventions](topic-conventions.md) | Standard ROS 2 topic names and frame conventions |
| [Drone Patterns](drone-patterns.md) | Offboard heartbeat, policy server, safety |
| [LLM Integration](llm-integration.md) | Prompt templates, schema validation, error handling |
| [Development Workflow](development-workflow.md) | Remote-first vs local-first analysis |

---

## Development Guidelines

- Prefer Python ROS 2 nodes (rclpy) initially for rapid iteration
- Every milestone needs a `make demo-*` command and README section
- Add a safety stop node (zero `cmd_vel` after N seconds of no input)
- Use rosbag2 recording helper scripts for all demos
- Start with 2D nav (slam_toolbox/AMCL); upgrade to 3D VSLAM once shipping demos
- Keep first arm loop simple: stable TF tree, good joint limits, reliable execution, repeatable grasp heuristics

---

## Version Pins

Avoid chasing latest. Pin to stable versions:

| Component | Version | Notes |
|-----------|---------|-------|
| Isaac Sim | 5.1.x | Stable baseline (6.0 is EDR/source-build-only) |
| Isaac Lab | 2.3.x | Built on Isaac Sim 5.1 |
| Python | 3.11 | Isaac Sim only; external nodes use system Python via DDS |
| ROS 2 | Jazzy | Ubuntu 24.04; Humble container for package compat |

---

## Tech Stack

| Layer | Primary | Fallback |
|-------|---------|----------|
| Ground sim | Isaac Sim + Isaac Lab | Gazebo |
| Drone sim | PX4 SITL + Gazebo | Isaac Sim + Pegasus |
| ROS 2 | Jazzy (Ubuntu 24.04) | Humble container |
| Autopilot | PX4 via uXRCE-DDS | ArduPilot via AP_DDS |
| Nodes | Python (rclpy) | C++ for performance-critical |
