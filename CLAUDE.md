# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A ROS 2-based robotics platform supporting:
- Rolling base (vision-powered RC rover)
- Robot arm (tabletop manipulation)
- Flying drone (camera + offboard autonomy)

With a shared LLM/VLM cognitive layer for scene understanding, goal interpretation, and task planning.

**Core constraint**: Everything runs in simulation first; swapping to real hardware should require only driver changes, not architecture rewrites.

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

## Phase 0: Mac-to-AWS Hello World

Before any robotics work, deploy and verify streaming connectivity.

### Quick Start (Terraform)
```bash
cd infra/terraform
terraform init
terraform apply \
  -var="my_ip_cidr=$(curl -s ifconfig.me)/32" \
  -var="key_name=your-key" \
  -var="isaac_sim_ami=ami-xxx"
```

### Connect
1. SSH: `ssh -i ~/.ssh/your-key.pem ubuntu@<ip>`
2. Verify GPU: `nvidia-smi` (should show L40s)
3. Start Isaac Sim: `./isaac-sim.sh --/app/livestream/enabled=true`
4. Mac WebRTC client → `<ip>:49100`
5. Verify ROS: `ros2 topic list` (should show `/clock`, `/tf`, cameras)

### Cost Control
```bash
./scripts/stop-instance.sh  # Stop when done (billing stops)
./scripts/start-instance.sh # Resume later
```

See `docs/setup/aws-mac-hello-world.md` for detailed guide.

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
  docs/
    setup/                # Setup guides (aws-mac-hello-world.md)
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

## ROS 2 Topic Conventions

### Core Time + Transforms
- `/clock` (sim publishes)
- `/tf`, `/tf_static`
- Frames: `map` → `odom` → `base_link` → ...

### Rolling Base
- Publish: `/odom`, `/imu/data`, `/joint_states`
- Subscribe: `/cmd_vel` (geometry_msgs/Twist)
- Cameras: `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_raw`, `/camera/depth/camera_info`
- Camera frames: `camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame`

### Manipulation (ros2_control + MoveIt)
- `/arm_controller/follow_joint_trajectory` (action)
- `/gripper_controller/command`
- Frames: `base_link` → `arm_base_link` → ... → `tool0` → `gripper_link`

### Drone (via uXRCE-DDS)
- Input: Vehicle odometry/state from PX4
- Output: OffboardControlMode heartbeat (≥2 Hz), trajectory setpoints
- **Frame warning**: PX4 is NED, ROS is ENU. Create a single `frame_bridge` node for conversions; nothing else does ad-hoc frame math.

### Capability API (stable across robots)
- `/capabilities/navigate_to_pose` (action)
- `/capabilities/align_to_object` (action)
- `/capabilities/pick`, `/capabilities/place` (actions)
- `/perception/object_pose`

## Drone-Specific Architecture

### Offboard Heartbeat Pattern
The autonomy node is part of the safety chain. Use this architecture:
1. **offboard_heartbeat** - Tiny, boring watchdog that maintains ≥2 Hz heartbeat + neutral setpoints
2. **smart_planner** - Publishes desired setpoints into a buffer the heartbeat node forwards
3. If smart node dies, heartbeat degrades gracefully (hold → land) instead of dropping offboard

### Offboard Requirements
- Must stream setpoints for ~1 second before switching to Offboard mode
- If stream drops below 2 Hz, PX4 exits offboard and triggers failsafe

## Policy Server Pattern (Learned Skills)

Swap policies like batteries without touching nav2/MoveIt/executive:

```
(perception) → ObjectPose → (executive BT) → AlignToObject action
                                    ↓
                          (observation_mux) → Observation → (policy_server) → cmd_vel
                                                                  ↓
                                                            (safety_filter) → cmd_vel
```

- **observation_mux**: Subscribes to policy inputs, publishes single `Observation` msg
- **policy_server**: Loads TorchScript/ONNX artifact, provides `SetPolicy.srv` for hot-swap
- **safety_filter**: Clamps velocities/deltas, enforces timeouts + e-stop, gates by allowed regions

The executive never knows if a skill is classical, learned, or hybrid—it just calls the action.

## LLM/VLM Integration Rules

1. Frame sampler runs at 1-2 Hz, downscales + compresses images
2. All VLM output validated against `schemas/scene_analysis.schema.json`
3. Planner output validated against `schemas/action_plan.schema.json`
4. Skill calls validated against `schemas/skill_call.schema.json`
5. Add timeouts to all VLM/LLM calls
6. Keep cognitive layer swappable (frontier API / local VLM / scripted fallback)

## Tech Stack

- **Ground sim**: Isaac Sim + Isaac Lab (primary) or Gazebo (fallback)
- **Drone sim**: PX4 SITL + Gazebo (gz) for sim-to-real path; Isaac Sim + Pegasus for photoreal/synthetic data
- **ROS 2**: Jazzy (Ubuntu 24.04) with Humble container for compatibility
- **Autopilot**: PX4 via uXRCE-DDS (primary) or ArduPilot via AP_DDS
- **Nodes**: Python (rclpy) first for speed; optimize later

## Cloud Workflow

- Keep ROS graph co-located (same instance or VPC). **Never run DDS over public internet.**
- Use WebRTC streaming for Mac thin client (requires NVENC; A100 won't work)
- Ports: TCP 49100, UDP 47998 for WebRTC; SSH + DCV for workstation access
- Automate stop-start (Isaac Automator or IaC) to control costs

## Demo Metrics (Track From Day One)

Each demo should log:
- Success rate
- Time-to-goal
- Collisions / interventions
- Offboard dropouts (drones)
- Grasp success (manipulation)

## Hardware Purchase Decision Framework

**Buy hardware when:**
- Demo 2 (autonomous navigation) runs end-to-end reliably in sim, OR
- Tabletop pick/place works and you're hitting reality-gap questions

**Don't buy when:**
- Still changing topic names, TF frames, robot descriptions weekly
- No rosbag2 replay workflow yet
- No defined success metrics

**Selection rubric**: ROS 2 native support, spare parts availability, sensor mounting friendliness, compute strategy (Jetson vs x86), payload/stability, community responsiveness.

## Development Guidelines

- Prefer Python ROS 2 nodes (rclpy) initially
- Every milestone needs a `make demo-*` command and README section
- Add a safety stop node (zero cmd_vel after N seconds of no input)
- Use rosbag2 recording helper scripts for all demos
- Start with 2D nav (slam_toolbox/AMCL); upgrade to 3D VSLAM once shipping demos
- Keep first arm loop simple: stable TF tree, good joint limits, reliable execution, repeatable grasp heuristics
