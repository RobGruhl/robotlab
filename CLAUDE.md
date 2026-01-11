# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A ROS 2-based robotics platform supporting:
- Rolling base (vision-powered RC rover)
- Robot arm (tabletop manipulation)
- Flying drone (camera + offboard autonomy)

With a shared LLM/VLM cognitive layer for scene understanding, goal interpretation, and task planning.

**Core constraint**: Everything runs in simulation first; swapping to real hardware should require only driver changes, not architecture rewrites.

## Claude Sync Channel

**`.claude/claude-sync.md`** is an async communication channel between Claude Code instances (AWS and laptop). Use it to:
- Share status updates and debugging findings
- Coordinate tasks across sessions
- Document what works/doesn't work

**Protocol:** Pull before reading, append new messages at the bottom with timestamp and source (e.g., `### [YYYY-MM-DD HH:MM] FROM-AWS`), push after writing.

## Build & Run Commands

```bash
make build          # Build ROS workspace
make test           # Run tests
make lint           # Run linters
make demo-smoke     # Minimal end-to-end graph test
make demo-base-teleop   # Teleop rolling base in sim
make demo-vlm-scene     # VLM scene analysis demo
make demo-arm-pickplace # Arm manipulation demo
make demo-drone-hover   # Drone offboard control demo
```

## Repository Structure

```
robotlab/
  ros_ws/src/
    robotlab_common/      # Shared messages, logging, time utils
    robotlab_bringup/     # Launch files + configs
    robotlab_base/        # Base control + nav wrappers
    robotlab_arm/         # MoveIt integration + pick/place
    robotlab_drone/       # PX4/ArduPilot integration + offboard
    robotlab_perception/  # Camera utils, frame sampling
    robotlab_scene_graph/ # Scene graph + memory
    robotlab_llm/         # VLM/LLM client + planner + schemas
    robotlab_executor/    # Skill registry, safety gate, behavior tree
  sim/
    isaac/                # Isaac Sim worlds, robots, scripts
    gazebo/               # Gazebo worlds, models, launch files
  schemas/                # JSON schemas for LLM output validation
  scripts/                # Demo + instance control scripts
  infra/
    terraform/            # AWS infrastructure (EC2, security groups)
    docker/               # Dockerfiles for ROS Jazzy/Humble
  docs/                   # Documentation (see below)
```

## Architecture Layers

1. **Robot I/O** - Sim or hardware drivers presenting standard ROS 2 topics (the only layer you swap when changing robots)
2. **Skills** - ROS 2 actions/services (NavigateToPose, Pick, Place, etc.)
3. **Scene Graph** - Persistent world model updated from perception
4. **LLM/VLM Cognitive** - Scene analysis + goal interpretation + planning
5. **Executor + Safety Gate** - Trust boundary enforcing schemas + constraints

## Non-Negotiable Principles

- **ROS 2 is the thin waist**: Every robot presents the same contract (sensors in, commands out, common TF tree)
- **LLM outputs are untrusted**: LLMs propose; executor validates schemas + enforces safety; LLMs never command raw motors
- **Drones require autopilot**: PX4 or ArduPilot owns stabilization/safety; ROS 2 runs "above" it via offboard control
- **Record everything**: Every demo must be rosbag2-loggable for replay debugging
- **Sim time from Day 1**: Always use `/clock` + `use_sim_time:=true` for nav2, TF, EKF, planners, executive

## Version Pins (Avoid Chasing Latest)

- **Isaac Sim**: 5.1.x (stable baseline; 6.0 is EDR/source-build-only)
- **Isaac Lab**: 2.3.x (built on Isaac Sim 5.1)
- **Python**: 3.11 only for Isaac Sim; run external ROS nodes via DDS to avoid environment pain
- **ROS 2**: Jazzy for modern GPU stack, Humble container for package compatibility

## Isaac Sim Management

**Before launching Isaac Sim**, always kill existing instances:
```bash
pkill -9 -f "isaac-sim|kit/kit"
```

**Launch with ROS 2 bridge working** (clean environment, no system ROS 2 paths):
```bash
cd /opt/IsaacSim && env -i HOME=$HOME DISPLAY=$DISPLAY \
  PATH=/usr/local/bin:/usr/bin:/bin \
  ROS_DISTRO=jazzy \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/opt/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib \
  ./isaac-sim.sh
```

**Why the clean environment?** Isaac Sim uses Python 3.11 exclusively; system ROS 2 Jazzy uses Python 3.12. Sourcing `/opt/ros/jazzy/setup.bash` before launching causes rclpy import failures.

**The Python 3.11 escape hatch:** Isaac Sim's internal ROS 2 bridge and external ROS 2 nodes communicate via DDS regardless of Python version. Run Isaac Sim with its internal Python 3.11 environment, run ROS 2 nodes externally with system Python 3.12. DDS handles transport - no shared Python needed.

**In your ROS 2 terminal** (separate from Isaac Sim):
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
ros2 topic list  # Should see /clock if ActionGraph is wired
```

**Important:** Raw USD robots dragged into the scene don't publish ROS 2 topics. You must add OmniGraph nodes (see `docs/setup/isaac-sim-reference.md` for step-by-step).

## Detailed Documentation

| Topic | Location |
|-------|----------|
| **Getting Started** | [docs/getting-started/](docs/getting-started/) - AWS deployment, first robot |
| **Architecture** | [docs/architecture/](docs/architecture/) - Topic conventions, drone patterns, LLM rules |
| **OmniGraph Reference** | [docs/omnigraph/](docs/omnigraph/) - Node catalog, ROS 2 patterns |
| **Roadmap** | [docs/roadmap/](docs/roadmap/) - Vision, backlog, hardware decisions |
| **Isaac Sim Setup** | [docs/setup/isaac-sim-reference.md](docs/setup/isaac-sim-reference.md) - Detailed usage guide |
| **Claude Code on AWS** | [docs/setup/claude-code-aws.md](docs/setup/claude-code-aws.md) - Remote development |
