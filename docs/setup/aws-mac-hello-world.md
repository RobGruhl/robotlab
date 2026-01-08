# Phase 0: Mac-to-AWS Hello World

Connect your MacBook Pro to an AWS instance running Isaac Sim with WebRTC streaming.

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

## Step 4: Start Isaac Sim with Streaming

```bash
# On the AWS instance (via SSH)

# Navigate to Isaac Sim installation
cd /opt/nvidia/isaac-sim  # or wherever installed on the AMI

# Start with WebRTC streaming enabled
./isaac-sim.sh --/app/livestream/enabled=true
```

Isaac Sim takes 2-5 minutes to load on first launch. You'll see logging output in the terminal.

## Step 5: Install Mac WebRTC Client

1. Download the **Isaac Sim WebRTC Streaming Client** from NVIDIA:
   - Go to [NVIDIA Isaac Sim Downloads](https://developer.nvidia.com/isaac-sim)
   - Download the macOS streaming client
2. Install the application
3. Note: Only **one client can connect at a time**

## Step 6: Connect from Mac

1. Open the Isaac Sim WebRTC Streaming Client
2. Enter the server address: `<public_ip>:49100`
3. Click Connect
4. You should see the Isaac Sim viewport streaming to your Mac

**Troubleshooting connection issues:**
- Verify ports 49100 (TCP) and 47998 (UDP) are open in security group
- Ensure Isaac Sim finished loading (check SSH terminal for "Ready" message)
- Try refreshing or reconnecting after 30 seconds

## Step 7: Verify ROS 2 Bridge

In a **new SSH terminal** (keep Isaac Sim running in the first):

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# List topics - you should see /clock, /tf, and camera topics
ros2 topic list
```

Expected output includes:
```
/clock
/tf
/tf_static
/camera/color/image_raw
/camera/color/camera_info
```

If no topics appear, ensure the ROS 2 bridge is enabled in your Isaac Sim scene.

## Step 8: Hello World - Camera Topic

```bash
# Echo a single camera message to verify the pipeline
ros2 topic echo /camera/color/image_raw --once
```

You should see image data with header, height, width, encoding fields.

**Congratulations!** Your Mac-to-AWS streaming pipeline is working.

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

### No WebRTC stream / connection refused
- Verify security group has ports 49100 (TCP) and 47998 (UDP) open
- Ensure Isaac Sim is running with `--/app/livestream/enabled=true`
- Wait 2-3 minutes for Isaac Sim to fully load
- Check that your IP hasn't changed (re-run Terraform if needed)

### Black screen in WebRTC client
- Isaac Sim may still be loading - wait and retry
- Check SSH terminal for error messages
- Restart Isaac Sim if needed

### ROS topics not appearing
- Ensure ROS 2 bridge is enabled in your Isaac Sim scene
- Source ROS 2: `source /opt/ros/jazzy/setup.bash`
- Check for Python version conflicts (Isaac Sim requires Python 3.11)

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
| Never run DDS over public internet | Keep ROS graph co-located on AWS. Stream video only. |
| A100 won't stream | Must use g6e.2xlarge (L40S has NVENC) |
| One client at a time | WebRTC limitation |
| Python 3.11 only | Isaac Sim constraint |
| Stop when done | ~$1.50/hr adds up fast |

## Next Steps

Once streaming works:
1. Load a robot scene in Isaac Sim
2. Enable the ROS 2 bridge for that scene
3. Verify camera and TF topics are publishing
4. Try teleop control with `ros2 topic pub` or a joystick node

See the main CLAUDE.md for the full robotics development workflow.

## Isaac Automator (Optional)

For more advanced automation (scheduled start/stop, multi-user, etc.), see:
- [Isaac Automator Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_cloud.html)
- Provides infrastructure-as-code templates for various cloud providers
- Supports upload/download of workspaces
- More sophisticated cost control options
