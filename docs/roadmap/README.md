# Roadmap

Vision, phases, and planning for the robotlab platform.

## North Star Vision

Build a single ROS 2-centric "robot brain" stack that can drive:

1. **Rolling base** - Vision-powered RC rover
2. **Robot arm** - Tabletop manipulation (mobile manipulation later)
3. **Flying drone** - Camera + offboard autonomy

With a shared LLM/VLM cognitive layer for:
- Scene understanding ("what's here / what changed / what's blocking?")
- Goal interpretation ("go to the red cone", "pick up the mug", "inspect the window")
- Task planning ("break down objective into skill calls")

**Key constraint:** Everything runs in simulation first. Swapping to real hardware is a bringup/driver change, not an architecture rewrite.

---

## Demo Ladder

Each milestone becomes a `make demo-*` command.

| Demo | Name | Description |
|------|------|-------------|
| D0 | Base Teleop | Teleop a simulated rolling base, view camera in RViz |
| D1 | Scene Description | VLM returns structured list of objects + obstacles |
| D2 | Natural Language Nav | "Go to the red cone" → robot navigates there |
| D3 | Pick and Place | "Pick up the mug" → arm executes in sim |
| D4 | Drone Hover | Takeoff → hover → land with ROS 2 offboard |
| D5 | Drone Inspection | "Inspect the window" → short inspection path + scene report |
| D6 | North Star Mission | Combined: drone scouts, rover navigates, arm manipulates |

---

## Current Status

**Phase 1 complete:** Jetbot + ROS 2 + Isaac Sim hello world working (D0 partial)

**Next steps:**
- Add `/odom` publisher for odometry feedback
- Add `/tf` publisher for transforms
- Add camera topics for perception
- Nav2 integration

---

## Phase Summary

| Phase | Goal | Sessions |
|-------|------|----------|
| 0 | Bootstrap: repo + Docker + Makefile + basic nodes | 1-2 |
| 1 | Rolling base sim + ROS 2 contract (D0) | 2-5 |
| 2 | VLM scene analysis + scene graph (D1) | 2-6 |
| 3 | Natural language navigation (D2) | 3-10 |
| 4 | Arm simulation + pick/place (D3) | 5-15 |
| 5 | Drone simulation + offboard (D4/D5) | 5-20 |
| 6 | Unified north star mission (D6) | TBD |
| 7 | Learning with Isaac Lab + LLM help | Optional |
| 8 | Sim-to-real readiness | Future |

---

## Tech Stack

| Layer | Primary | Fallback |
|-------|---------|----------|
| Ground sim | Isaac Sim + Isaac Lab | Gazebo |
| Drone sim | PX4 SITL + Gazebo | Isaac Sim + Pegasus |
| ROS 2 | Jazzy (Ubuntu 24.04) | Humble container |
| Autopilot | PX4 via uXRCE-DDS | ArduPilot via AP_DDS |

---

## Demo Metrics

Track from day one:
- Success rate
- Time-to-goal
- Collisions / interventions
- Offboard dropouts (drones)
- Grasp success (manipulation)

---

## Hardware Purchase Framework

**Buy hardware when:**
- Demo 2 (autonomous navigation) runs end-to-end reliably in sim, OR
- Tabletop pick/place works and you're hitting reality-gap questions

**Don't buy when:**
- Still changing topic names, TF frames, robot descriptions weekly
- No rosbag2 replay workflow yet
- No defined success metrics

**Selection rubric:**
- ROS 2 native support
- Spare parts availability
- Sensor mounting friendliness
- Compute strategy (Jetson vs x86)
- Payload/stability
- Community responsiveness

---

## Reference URLs

```
Isaac Lab release notes:
https://isaac-sim.github.io/IsaacLab/main/source/refs/release_notes.html

PX4 uXRCE-DDS bridge:
https://docs.px4.io/main/en/middleware/uxrce_dds

PX4 Offboard control:
https://docs.px4.io/main/en/ros2/offboard_control

Pegasus Simulator:
https://pegasussimulator.github.io/PegasusSimulator/

ArduPilot ROS 2:
https://ardupilot.org/dev/docs/ros2.html
```

---

See [backlog.md](backlog.md) for current work items and hardware decisions.
