# Claude Sync Channel

Async communication channel between Claude Code instances (AWS and laptop).

## Protocol

- Append new messages at the bottom with timestamp and source
- Pull before reading, push after writing
- Format: `### [YYYY-MM-DD HH:MM] FROM-LOCATION`

## Current System State

| Item | Value |
|------|-------|
| **Status** | Jetbot + ROS 2 + Isaac Sim working end-to-end |
| **Last verified** | 2026-01-11 05:33 |
| **Ubuntu** | 24.04 |
| **Isaac Sim** | 5.0.0-rc.45 at `/opt/IsaacSim/` |
| **ROS 2** | Jazzy at `/opt/ros/jazzy/` |
| **RMW** | `rmw_fastrtps_cpp` (both sides) |
| **Scene file** | `sim/isaac/jetbot_hello_world.usd` |

**Working Isaac Sim launch command:**
```bash
cd /opt/IsaacSim && env -i HOME=$HOME DISPLAY=$DISPLAY \
  PATH=/usr/local/bin:/usr/bin:/bin \
  ROS_DISTRO=jazzy \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/opt/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib \
  ./isaac-sim.sh
```

**Test command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10
```

## Next Steps

- Add `/odom` publisher for odometry feedback
- Add `/tf` publisher for transforms
- Add camera topics for perception
- Nav2 integration

---

*See `.claude/sync-history/` for archived debugging sessions.*

---

## Active Messages

(Append new messages below this line)
