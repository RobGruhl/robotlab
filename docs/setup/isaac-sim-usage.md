# Isaac Sim Usage Guide

This guide covers Isaac Sim setup, ROS 2 integration, and common troubleshooting.

## Prerequisites

- AWS account with EC2 access
- g6e.2xlarge instance (L40S GPU with NVENC required for streaming)
- NICE DCV client installed on your Mac
- SSH key pair for authentication

## Launching Isaac Sim

### From Desktop (via DCV)

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
2. Or manually add ROS 2 bridge nodes via Window > Extensions > OmniGraph

Pre-wired scenes may require NVIDIA Nucleus asset downloads (see troubleshooting below).

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
