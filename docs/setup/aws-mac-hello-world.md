# Phase 0: Mac-to-AWS Hello World

Connect your MacBook Pro to an AWS instance running Isaac Sim with NICE DCV streaming.

## Prerequisites

- AWS account with EC2 access
- AWS CLI installed and configured (`brew install awscli && aws configure`)
- Terraform installed (`brew install terraform`)
- SSH key pair created in AWS (or create one: EC2 → Key Pairs → Create)

## Step 1: Find the Isaac Sim AMI

1. Go to [AWS Marketplace](https://aws.amazon.com/marketplace)
2. Search for **"NVIDIA Isaac Sim Development Workstation"**
3. Click **Continue to Subscribe**
4. Accept terms and wait for subscription to activate (~2 minutes)
5. Click **Continue to Configuration**
6. Select your region (e.g., `us-west-2`)
7. Note the **AMI ID** (e.g., `ami-0abc123def456...`)

## Step 2: Deploy with Terraform

```bash
cd infra/terraform

# Initialize Terraform
terraform init

# Deploy (replace values with your own)
terraform apply \
  -var="my_ip_cidr=$(curl -s ifconfig.me)/32" \
  -var="key_name=your-ssh-key-name" \
  -var="isaac_sim_ami=ami-xxxxxxxxxxxxxxxxx" \
  -var="region=us-west-2"
```

Terraform will show you what it will create. Type `yes` to confirm.

After completion, note the outputs:
- `public_ip` - Your instance's IP address
- `ssh_command` - Ready-to-use SSH command
- `webrtc_url` - WebRTC streaming URL

## Step 3: SSH and Verify GPU

```bash
# Use the SSH command from Terraform output
ssh -i ~/.ssh/your-key.pem ubuntu@<public_ip>

# Verify GPU is available
nvidia-smi
```

You should see the **NVIDIA L40S** GPU. If not, the instance type is wrong.

## Step 4: Start Isaac Sim

Isaac Sim is pre-installed on the AMI. For ROS 2 integration, use the clean environment launch:

```bash
# On the AWS instance (via DCV desktop, not SSH)

# Kill any existing instances
pkill -9 -f "isaac-sim|kit/kit"

# Launch with ROS 2 bridge enabled
cd /opt/IsaacSim && env -i HOME=$HOME DISPLAY=$DISPLAY \
  PATH=/usr/local/bin:/usr/bin:/bin \
  ROS_DISTRO=jazzy \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/opt/IsaacSim/exts/isaacsim.ros2.bridge/jazzy/lib \
  ./isaac-sim.sh
```

Isaac Sim takes 5-10 minutes to load on first launch (shader compilation). Click "Wait" on any "not responding" dialogs.

## Step 5: Install NICE DCV Client

1. Download the **NICE DCV Client** from: https://download.nice-dcv.com/
2. Install the macOS application
3. Note: Only **one client can connect at a time**

## Step 6: Connect from Mac

1. Open the NICE DCV Client
2. Enter the server address: `<public_ip>:8443`
3. Accept the self-signed certificate warning
4. Login with username `ubuntu` and the password you set in Step 3
5. You should see the Ubuntu desktop with Isaac Sim

**Troubleshooting connection issues:**
- Verify port 8443 (TCP) is open in security group
- Ensure you set the ubuntu password (`sudo passwd ubuntu`)
- Try using `https://` prefix explicitly

## Step 7: Verify ROS 2 Bridge

In a **new SSH terminal** (keep Isaac Sim running):

```bash
# Source ROS 2 with RMW fix
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# List topics
ros2 topic list
```

**Important:** You won't see `/clock` until you add an OmniGraph with ROS 2 publisher nodes. Raw USD robots don't publish topics automatically.

To add a minimal `/clock` publisher:
1. In Isaac Sim: Window → Visual Scripting → Action Graph
2. Add nodes: OnPlaybackTick, IsaacReadSimulationTime, ROS2PublishClock
3. Connect: OnPlaybackTick.Tick → ROS2PublishClock.ExecIn
4. Connect: ReadSimulationTime.SimulationTime → ROS2PublishClock.TimeStamp
5. Press Play

See `docs/setup/isaac-sim-usage.md` for detailed OmniGraph instructions.

## Step 8: Hello World - Clock Topic

```bash
# Echo the clock to verify Isaac Sim → ROS 2 communication
ros2 topic echo /clock --once
```

You should see:
```
clock:
  sec: 42
  nanosec: 500001173
```

**Congratulations!** Your Mac-to-AWS Isaac Sim + ROS 2 pipeline is working.

## Step 9: Cost Control

**g6e.2xlarge costs approximately $1.50/hour.** Always stop the instance when not in use.

```bash
# Stop the instance (run from your Mac, not SSH)
./scripts/stop-instance.sh

# Later, resume work
./scripts/start-instance.sh
```

The stop script:
- Stops the EC2 instance
- Preserves all data on the EBS volume
- Stops billing (you only pay for EBS storage, ~$0.08/GB/month)

## Troubleshooting

### DCV connection refused or timeout
- Verify security group has port 8443 (TCP) open
- Ensure you set ubuntu password: `sudo passwd ubuntu`
- Wait 2-3 minutes after instance start for DCV to initialize
- Check that your IP hasn't changed (re-run Terraform if needed)

### Black screen in DCV client
- Isaac Sim may still be loading shaders (5-10 min on first run)
- Check SSH terminal for progress messages
- Click "Wait" on any "not responding" dialogs

### ROS topics not appearing
- **Most common issue:** Missing OmniGraph wiring - raw robots don't publish topics
- **Second most common:** RMW mismatch - must set `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Don't source system ROS 2 before launching Isaac Sim (causes Python conflicts)
- Ensure Play button is pressed in Isaac Sim

### "NVENC not available" or streaming fails
- This means you're on the wrong GPU type
- Confirm instance is g6e.2xlarge (L40S has NVENC)
- A100 GPUs do NOT support streaming (no NVENC)

### IP address changed after stop/start
- Normal behavior unless you use an Elastic IP
- Re-run `./scripts/start-instance.sh` to see new IP
- Consider adding an Elastic IP for persistent addressing

## Key Warnings

| Warning | Details |
|---------|---------|
| Never run DDS over public internet | Keep ROS graph co-located on AWS. Stream video only via DCV. |
| A100 won't stream | Must use g6e.2xlarge (L40S has NVENC) |
| One DCV client at a time | DCV limitation |
| Python 3.11 in Isaac Sim | Use DDS to communicate with system ROS 2 (Python 3.12) |
| RMW must match | Set `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in ROS 2 terminal |
| Stop when done | ~$2.40/hr for g6e.2xlarge adds up fast |

## Next Steps

Once `/clock` is visible:
1. Add more OmniGraph nodes for `/tf`, `/cmd_vel`, cameras
2. Drag a robot (e.g., Jetbot) into the scene
3. Wire up ROS2SubscribeTwist to control the robot with `/cmd_vel`
4. Try teleop: `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"`

See `docs/setup/isaac-sim-usage.md` for detailed OmniGraph instructions.
See the main CLAUDE.md for the full robotics development workflow.
