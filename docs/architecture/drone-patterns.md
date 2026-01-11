# Drone Architecture Patterns

Patterns for safe drone autonomy with PX4/ArduPilot via ROS 2.

## Offboard Heartbeat Pattern

The autonomy node is part of the safety chain. Use this architecture to ensure graceful degradation:

```
                    +-------------------+
                    |  smart_planner    |
                    | (VLM/LLM/Nav)     |
                    +--------+----------+
                             |
                             v (setpoint buffer)
                    +--------+----------+
                    | offboard_heartbeat|  <-- Tiny, boring watchdog
                    | (≥2 Hz heartbeat) |
                    +--------+----------+
                             |
                             v
                    +--------+----------+
                    |      PX4          |
                    | (stabilization)   |
                    +-------------------+
```

**Nodes:**

1. **offboard_heartbeat** - Tiny, boring watchdog that maintains ≥2 Hz heartbeat + neutral setpoints. Never crashes.

2. **smart_planner** - Publishes desired setpoints into a buffer that the heartbeat node forwards.

3. **Graceful degradation** - If smart node dies, heartbeat degrades gracefully (hold position → land) instead of dropping offboard.

---

## Offboard Requirements

PX4 has strict requirements for offboard control:

| Requirement | Value | Notes |
|-------------|-------|-------|
| Setpoint rate | ≥2 Hz | Stream must not drop below this |
| Warmup period | ~1 second | Stream setpoints before switching to Offboard mode |
| Failsafe trigger | <2 Hz | PX4 exits offboard and triggers failsafe |

**Key insight:** If the setpoint stream drops below 2 Hz, PX4 will exit offboard mode automatically. This is a safety feature, not a bug.

---

## Policy Server Pattern (Learned Skills)

Swap policies like batteries without touching nav2/MoveIt/executive:

```
(perception) --> ObjectPose --> (executive BT) --> AlignToObject action
                                        |
                                        v
                              (observation_mux) --> Observation --> (policy_server) --> cmd_vel
                                                                          |
                                                                          v
                                                                    (safety_filter) --> cmd_vel
```

**Components:**

| Node | Role |
|------|------|
| **observation_mux** | Subscribes to policy inputs, publishes single `Observation` msg |
| **policy_server** | Loads TorchScript/ONNX artifact, provides `SetPolicy.srv` for hot-swap |
| **safety_filter** | Clamps velocities/deltas, enforces timeouts + e-stop, gates by allowed regions |

The executive never knows if a skill is classical, learned, or hybrid—it just calls the action.

---

## Frame Bridge

PX4 uses NED (North-East-Down), ROS uses ENU (East-North-Up).

**Pattern:** Create a single `frame_bridge` node that handles all conversions. Nothing else does ad-hoc frame math.

```python
# frame_bridge node pseudocode
def px4_to_ros(ned_pose):
    """NED -> ENU: swap x/y, negate z"""
    return Pose(
        x=ned_pose.y,
        y=ned_pose.x,
        z=-ned_pose.z
    )

def ros_to_px4(enu_pose):
    """ENU -> NED: swap x/y, negate z"""
    return Pose(
        x=enu_pose.y,
        y=enu_pose.x,
        z=-enu_pose.z
    )
```

---

## Safety Considerations

1. **Autopilot owns stabilization** - PX4/ArduPilot handles low-level control. ROS 2 runs "above" it via offboard mode.

2. **Never bypass the autopilot** - Don't send raw motor commands. Always use the offboard interface.

3. **Geofence** - Configure PX4 geofence parameters before outdoor flight.

4. **Kill switch** - Always have a hardware kill switch that cuts power to motors.

5. **Return-to-launch** - Configure RTL behavior for link loss.

---

## Recommended Simulators

| Simulator | Use Case | Notes |
|-----------|----------|-------|
| PX4 SITL + Gazebo | Sim-to-real fidelity | Standard for PX4 development |
| Isaac Sim + Pegasus | Photorealistic rendering | Synthetic data generation |
| AirSim | Familiar to ML researchers | Microsoft, less maintained |

For production, use PX4 SITL + Gazebo to ensure sim-to-real transfer of control parameters.
