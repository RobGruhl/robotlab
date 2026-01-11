# Backlog

Current work items and hardware research, prioritized by sim-to-real readiness.

---

## Phase 1: Current Focus

**Goal:** Hello World - drive a simulated robot with ROS 2 `/cmd_vel`

- [x] Isaac Sim + ROS 2 bridge working (`/clock` visible)
- [x] Jetbot responding to `/cmd_vel`
- [x] Add `/odom` publisher (via IsaacComputeOdometry → ROS2PublishOdometry)
- [x] Add `/tf` publisher (world → chassis → wheels)
- [ ] Chase camera (third-person follow cam) - **blocked on physics transform API**
- [ ] Add camera topics
- [ ] Nav2 integration

**Jetbot ActionGraph script:** `scripts/create_jetbot_graph.py`
**OmniGraph docs:** `docs/omnigraph/jetbot-graph.md`

### Open Technical Questions

See `.claude/claude-sync.md` for detailed research requests. Key blockers:

1. **Real-time physics transforms:** Can't read prim position during physics simulation
2. **Callback patterns:** Which subscription type sees physics state?
3. **Programmatic ground plane:** Physics collision not working when created via Python

---

## Phase 2: LIMO Cobot (Mobile Manipulator)

**Status:** Research complete, URDF cloned, ready when Phase 1 done

**Hardware:** AgileX LIMO Cobot (~$4,499)
- LIMO PRO base (Orin Nano, lidar, depth camera, IMU)
- myCobot 280 M5 arm (6-DOF, 280mm reach, 250g payload)
- Multi-mode drive: differential, Ackermann, tracked, omni
- Full ROS 2 stack with Nav2

**Simulation Assets:**
- LIMO URDF: `Limo-Isaac-Sim/limo_description/urdf/limo_base.urdf`
- myCobot URDF: https://github.com/elephantrobotics/mycobot_ros2

**Import Settings (from AgileX docs):**
- Uncheck "Fix Base Link"
- Joint Drive Type: Velocity
- Joint Drive Strength: 1000 (damping)
- Wheel Distance: 0.16m
- Wheel Radius: 0.025m

**Resources:**
- [Limo-Isaac-Sim](https://github.com/agilexrobotics/Limo-Isaac-Sim)
- [limo_ros2](https://github.com/agilexrobotics/limo_ros2)
- [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2)

---

## Phase 3: Scout Drone (SLAM + Aerial Recon)

**Status:** Research complete, waiting for ground robot foundation

**Concept:** Mars Rover style - LIMO carries a drone that scouts ahead for mapping and recon

**Hardware Options:**

| Drone | Price | Key Features | Best For |
|-------|-------|--------------|----------|
| **ModalAI Starling 2** | ~$2,500 | VOXL 2, PX4, visual SLAM, 40-55 min flight | Primary choice |
| **Starling 2 Max** | ~$3,500 | Longer flight time, more sensors | Extended missions |
| **DEXI PX4 Kit** | ~$500 | Educational, smaller, ROS capable | Budget learning |

**Why Starling:**
- Built for visual SLAM and indoor autonomy
- VOXL 2: Qualcomm QRB5165 with ~15 TOPS AI
- Ships with VIO, obstacle avoidance, 3D mapping software
- PX4 + ROS 2 native

**Simulation:**
- Use Pegasus Simulator (Isaac Sim drone framework)
- PX4 SITL with ROS 2
- Model Starling-class quad with typical sensors

**Integration with LIMO:**
- Drone launches from flat surface on/near LIMO
- Runs visual-SLAM scouting missions
- Streams maps + camera data back via ROS 2
- Central LLM/planning node coordinates both

**Resources:**
- [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator)
- [PX4 Starling Docs](https://docs.px4.io/main/en/complete_vehicles_mc/modalai_starling)
- [ModalAI VOXL SDK](https://docs.modalai.com/voxl-sdk/)

---

## Future Ideas

- **Swarm coordination:** Multiple LIMO-class bases with shared map
- **Arm manipulation:** Pick/place with myCobot, integrate MoveIt 2
- **LLM integration:** VLM scene understanding → task planning → skill execution
- **Real hardware:** Order LIMO Cobot once sim workflow is solid

---

## Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-01-11 | Start with Jetbot, not LIMO | Known quantity in Isaac Sim, faster to hello world |
| 2026-01-11 | LIMO Cobot as target hardware | Best value mobile manipulator under $5k |
| 2026-01-11 | Starling 2 for scout drone | PX4 + ROS 2 + visual SLAM, Isaac Sim compatible via Pegasus |
| 2026-01-11 | Two-layer deadman switch | Instance-level (SSH/DCV/GPU monitor) + AWS-level (nightly Lambda) prevents runaway costs |
| 2026-01-11 | IsaacComputeOdometry → ROS2PublishOdometry | Isaac Sim 5.0 API change - ROS2 node doesn't have chassisPrim input |
