# Isaac Sim Usage Guide

This guide covers Isaac Sim setup, ROS 2 integration, and common troubleshooting.

## Prerequisites

- AWS account with EC2 access
- g6e.2xlarge instance (L40S GPU with NVENC required for streaming)
- NICE DCV client installed on your Mac
- SSH key pair for authentication

## Launching Isaac Sim

### Working Configuration (ROS 2 Bridge Enabled)

**Always kill existing instances first:**
```bash
pkill -9 -f "isaac-sim|kit/kit"
```

**Launch with clean environment** (required for ROS 2 bridge to work):
```bash
cd /opt/IsaacSim && env -i HOME=$HOME DISPLAY=$DISPLAY \
  PATH=/usr/local/bin:/usr/bin:/bin \
  ROS_DISTRO=jazzy \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/opt/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib \
  ./isaac-sim.sh
```

**Why each variable:**
- `env -i` - Start with clean environment (no inherited ROS 2 paths)
- `ROS_DISTRO=jazzy` - Tell Isaac Sim which ROS 2 distro to use
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` - Use FastDDS (Isaac Sim's default)
- `LD_LIBRARY_PATH` - Point to Isaac Sim's internal ROS 2 libraries (Python 3.11 compatible)

### Simple Launch (No ROS 2)

If you don't need ROS 2 integration:
```bash
/opt/IsaacSim/isaac-sim.sh
```

### With Selector UI

```bash
/opt/IsaacSim/isaac-sim.selector.sh
```

### First Launch Notes

- **Shader compilation takes 5-10 minutes** on first run
- You will see "not responding" dialogs - click "Wait", don't force quit
- Once compiled, shaders are cached and future launches are fast
- Watch the terminal for progress messages

## ROS 2 Integration

### Critical Constraint: Python 3.11 Only

Isaac Sim is compatible with **Python 3.11 only**. This creates a version mismatch:
- Isaac Sim: Python 3.11 with bundled ROS 2 libraries
- ROS 2 Jazzy (Ubuntu 24.04): Python 3.12
- ROS 2 Humble (Ubuntu 22.04): Python 3.10

**Solution**: Run Isaac Sim and external ROS 2 nodes as separate processes. DDS handles transport regardless of Python version - they communicate over the network layer, not through Python imports.

### DDS/RMW Configuration (CRITICAL)

Isaac Sim's bundled ROS 2 and system ROS 2 Jazzy use **different DDS middleware by default**:
- Isaac Sim: FastDDS (rmw_fastrtps_cpp)
- ROS 2 Jazzy: CycloneDDS (rmw_cyclonedds_cpp)

**They must use the same RMW implementation to see each other's topics.**

#### Fix: Match RMW in Your Jazzy Terminal

```bash
# Before running ros2 commands
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# Now you should see Isaac Sim topics
ros2 topic list
```

#### Diagnostic Commands

```bash
# Check what RMW is in use
ros2 doctor --report | grep RMW
echo $RMW_IMPLEMENTATION

# Check domain ID
echo $ROS_DOMAIN_ID
```

#### Alternative: Force CycloneDDS on Both Sides

If you prefer CycloneDDS, you can configure Isaac Sim to use it instead. Check Isaac Sim's ROS 2 settings or environment variables.

### Environment Isolation

**Do NOT source system ROS 2 in the terminal that launches Isaac Sim.**

- Isaac Sim has its own ROS 2 environment
- Mixing environments causes conflicts
- Keep terminals separate:
  - Terminal 1: Launch Isaac Sim (no ROS 2 sourced)
  - Terminal 2: `source /opt/ros/jazzy/setup.bash` + ROS 2 commands

### Raw USD Robots Need OmniGraph Wiring

Dragging a robot USD file into the scene creates a **physics object only**. It won't publish ROS 2 topics automatically.

To get ROS 2 topics (`/clock`, `/tf`, `/cmd_vel`, cameras):
1. Use pre-wired sample scenes that include OmniGraph action graphs
2. Or manually add ROS 2 bridge nodes (see below)

Pre-wired scenes may require NVIDIA Nucleus asset downloads (see troubleshooting below).

### Adding ROS 2 Publishers via OmniGraph (Step-by-Step)

Minimal example: Add a `/clock` publisher to verify ROS 2 bridge is working.

**1. Open Action Graph Editor**
- Window → Visual Scripting → Action Graph
- Click "New Action Graph" (or select existing one and click "Edit Action Graph")
- The graph can be created under World or at root level

**2. Add Nodes** (right-click in graph canvas → search):
- `On Playback Tick` (triggers every simulation frame)
- `Isaac Read Simulation Time` (reads current sim time)
- `ROS2 Publish Clock` (publishes to /clock topic)

**3. Connect Nodes**
- Drag from `On Playback Tick` → `Tick` output to `ROS2 Publish Clock` → `Exec In`
- Drag from `Isaac Read Simulation Time` → `Simulation Time` to `ROS2 Publish Clock` → `Time Stamp`

**4. Press Play** (toolbar play button)

**5. Verify in ROS 2 Terminal**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list          # Should show /clock
ros2 topic echo /clock --once  # Should show sim time
```

**Common OmniGraph nodes for robots:**
- `ROS2 Publish Transform Tree` - publishes /tf
- `ROS2 Subscribe Twist` - subscribes to /cmd_vel
- `ROS2 Publish Odometry` - publishes /odom
- `ROS2 Publish Camera Info` + `ROS2 Publish Image` - camera topics

### Driving a Robot with /cmd_vel (Differential Drive)

Control a wheeled robot (Jetbot, LIMO, etc.) via ROS 2 Twist messages.

**Nodes needed (6 total):**
1. `On Playback Tick`
2. `ROS2 Subscribe Twist`
3. `Break 3-Vector` (for linear velocity)
4. `Break 3-Vector` (for angular velocity)
5. `Differential Controller`
6. `Articulation Controller`

**Connections:**

| From Node | From Port | To Node | To Port |
|-----------|-----------|---------|---------|
| On Playback Tick | Tick | ROS2 Subscribe Twist | Exec In |
| ROS2 Subscribe Twist | Exec Out | Articulation Controller | Exec In |
| ROS2 Subscribe Twist | Linear Velocity | Break 3-Vector (1st) | Vector |
| ROS2 Subscribe Twist | Angular Velocity | Break 3-Vector (2nd) | Vector |
| Break 3-Vector (1st) | X | Differential Controller | Linear Velocity |
| Break 3-Vector (2nd) | Z | Differential Controller | Angular Velocity |
| Differential Controller | Velocity Command | Articulation Controller | Velocity Command |

**Node Configuration:**

| Node | Property | Value (Jetbot) |
|------|----------|----------------|
| ROS2 Subscribe Twist | Topic Name | `/cmd_vel` |
| Differential Controller | Wheel Radius | `0.0325` |
| Differential Controller | Wheel Distance | `0.1125` |
| Articulation Controller | Robot Path | `/World/jetbot` |

**Test commands:**
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10

# Spin in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Troubleshooting

### Cache Permission Errors

**Error**: `Failed to create local file data store at '/opt/IsaacSim/kit/cache'`

**Fix**:
```bash
sudo mkdir -p /opt/IsaacSim/kit/cache
sudo chown -R ubuntu:ubuntu /opt/IsaacSim/kit/cache
```

### Isaac Sim Crashes During apt install

**Cause**: Installing packages while Isaac Sim is running can update shared libraries it depends on.

**Fix**: Always close Isaac Sim before running `apt install` or `apt upgrade`. Restart Isaac Sim after package installations.

### NVIDIA Nucleus Asset Downloads Hang

**Symptom**: Standalone Python examples stuck on "carb.tasking" or asset downloads.

**Workarounds**:
- Use local assets instead of Nucleus cloud assets
- Check network connectivity to NVIDIA servers
- Look for local sample USD scenes in `/opt/IsaacSim/` that don't require downloads

### ROS 2 Topics Not Visible

If `ros2 topic list` only shows `/parameter_events` and `/rosout`:

1. **Check RMW match** (see DDS/RMW Configuration above)
2. **Check Domain ID**: Both sides must use same `ROS_DOMAIN_ID`
3. **Check environment isolation**: Don't mix Isaac Sim and system ROS 2 environments
4. **Ensure Isaac Sim is playing**: Topics only publish when simulation is running (Play button pressed)
5. **Verify ROS 2 bridge is enabled**: Window > Extensions > search "ROS2"

### "Not Responding" During Startup

**Normal behavior** during first launch - Isaac Sim is compiling shaders.
- Click "Wait" on the dialog
- Watch terminal for progress
- Can take 5-10 minutes on first run
- Subsequent launches are much faster (shaders cached)

### WebRTC Streaming Issues

- Only **one client** can connect at a time
- A100 GPUs **cannot stream** (no NVENC encoder) - must use L40S (g6e instances)
- Required ports: TCP 49100, UDP 47998

### QoS Configuration

High-rate sim cameras can cause memory issues without proper QoS configuration.

For nav2/perception pipelines:
- Use throttling to reduce camera publish rate
- Use compressed image transport
- Configure appropriate QoS profiles (reliability, history depth)

## Reference URLs

- NVIDIA Forum - ROS2 bridge communication: https://forums.developer.nvidia.com/t/ros2-bridge-communication-problem/246195
- NVIDIA Forum - ROS 2 bridge issue: https://forums.developer.nvidia.com/t/ros-2-bridge-issue/229916
- Fast-DDS GitHub - Isaac Sim fix: https://github.com/eProsima/Fast-DDS/issues/3000
- NVIDIA troubleshooting docs: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_ros_communication.html

## AMI Quick Reference

From NVIDIA's Isaac Sim AMI documentation:

1. Navigate to AWS Marketplace and subscribe to "NVIDIA Isaac Sim Development Workstation"
2. Set instance type to **g6e.2xlarge** (required)
3. Configure security group with ports 22 (SSH) and 8443 (DCV)
4. After launch, SSH in and set password: `sudo passwd ubuntu`
5. Connect with NICE DCV client to `https://<ip>:8443`
