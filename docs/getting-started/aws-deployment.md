# AWS Deployment Guide

Deploy Isaac Sim on AWS with NICE DCV streaming to your Mac.

## Quick Version (5 Commands)

For experienced users who know what they're doing:

```bash
cd infra/terraform && terraform init

# Create terraform.tfvars with your values:
# my_ip_cidr = "YOUR_IP/32", key_name = "robotlab",
# isaac_sim_ami = "ami-0a58578db493e70fb", instance_type = "g6e.2xlarge"

terraform apply
ssh -i ~/.ssh/robotlab.pem ubuntu@$(terraform output -raw public_ip)
# Set password: sudo passwd ubuntu
# Connect DCV: https://<ip>:8443
```

**Critical:** Must use `g6e.2xlarge` (L40S GPU). Other GPU types won't work.

---

## What You'll Have When Done

```
YOUR MAC                                    AWS CLOUD
+---------------------+                    +---------------------+
|                     |                    |                     |
|  Terminal           |                    |                     |
|  Claude Code        |  (local control)   |                     |
|                     |                    |                     |
+---------------------+                    +---------------------+
|                     |                    |  +--------------+   |
|                     |    NICE DCV        |  | Ubuntu Desktop|  |
|  DCV Client         |<===(streaming)==== |  | +----------+ |   |
|  (full desktop!)    |   (port 8443)      |  | |Isaac Sim | |   |
|                     |                    |  | +----------+ |   |
|                     |                    |  | + Terminal   |   |
|                     |                    |  +--------------+   |
+---------------------+                    +---------------------+
```

**Key insight:** You see a **full remote desktop** via NICE DCV, not just a video stream. You can open terminals, run Isaac Sim with full GUI, and use all Ubuntu apps.

---

## Prerequisites Checklist

Before starting, verify each tool on your Mac.

### AWS CLI

```bash
aws --version
# Expected: aws-cli/2.x.x ...

# If missing:
brew install awscli
aws configure  # Enter Access Key, Secret Key, region (us-west-2), output (json)
```

### Terraform

```bash
terraform --version
# Expected: Terraform v1.x.x

# If missing:
brew install terraform
```

### SSH Key Pair in AWS

```bash
aws ec2 describe-key-pairs --region us-west-2
# Expected: JSON with at least one KeyName

# If empty, create one:
aws ec2 create-key-pair \
  --key-name robotlab \
  --region us-west-2 \
  --query 'KeyMaterial' \
  --output text > ~/.ssh/robotlab.pem
chmod 400 ~/.ssh/robotlab.pem
```

### Claude API Key in Secrets Manager (for Claude Code on AWS)

```bash
aws secretsmanager describe-secret --region us-west-2 --secret-id robotlab/claude-api-key
# If missing, see docs/setup/claude-code-aws.md
```

---

## Step 1: Find the Isaac Sim AMI

```bash
aws ec2 describe-images \
  --region us-west-2 \
  --owners aws-marketplace \
  --filters "Name=name,Values=*Isaac*Sim*" \
  --query 'Images[*].[ImageId,Name,CreationDate]' \
  --output table
```

Note the AMI ID (e.g., `ami-0a58578db493e70fb`).

**If empty:** Subscribe at AWS Marketplace → search "NVIDIA Isaac Sim Development Workstation" → Accept terms → Wait for activation.

---

## Step 2: Create Terraform Configuration

```bash
cd ~/Projects/robotlab/infra/terraform

cat > terraform.tfvars << 'EOF'
my_ip_cidr      = "YOUR_IP/32"
key_name        = "robotlab"
isaac_sim_ami   = "ami-0a58578db493e70fb"
region          = "us-west-2"
use_spot        = false
instance_type   = "g6e.2xlarge"
EOF

# Fill in your IP
MY_IP=$(curl -s -4 ifconfig.me)
sed -i '' "s|YOUR_IP|$MY_IP|g" terraform.tfvars

# Verify
cat terraform.tfvars
```

**Critical settings:**
- `instance_type = "g6e.2xlarge"` - **Required**. Other GPU types won't work.
- `use_spot = false` - **Required**. Spot not supported for this AMI.

---

## Step 3: Deploy Infrastructure

```bash
terraform init
terraform plan   # Review what will be created
terraform apply  # Type 'yes' when prompted
```

Wait ~2-3 minutes. Note the `public_ip` output.

**Common errors:**
- "UnsupportedOperation" → Wrong instance type, must be g6e
- "VcpuLimitExceeded" → Request quota increase for GPU instances

---

## Step 4: SSH and Verify GPU

```bash
ssh -i ~/.ssh/robotlab.pem ubuntu@<public_ip>
nvidia-smi
```

**Expected:** NVIDIA L40S GPU visible with ~46GB memory.

---

## Step 5: Set Ubuntu Password (Required for DCV)

```bash
sudo passwd ubuntu
# Enter password twice - remember this for DCV login
```

---

## Step 6: Install NICE DCV Client

1. Download from https://download.nice-dcv.com/
2. Install the macOS application
3. Open NICE DCV Viewer

---

## Step 7: Connect with DCV

1. Enter: `https://<public_ip>:8443`
2. Accept certificate warning → **Trust**
3. Login: `ubuntu` / your password

**You should see:** Full Ubuntu desktop.

---

## Step 8: Launch Isaac Sim

In a terminal on the remote desktop:

```bash
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

**First launch takes 5-10 minutes** (shader compilation). Click "Wait" on "not responding" dialogs.

---

## Step 9: Verify ROS 2 Bridge

In a **separate SSH terminal**:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
ros2 topic list
```

**Note:** You won't see `/clock` until you add OmniGraph nodes. See `docs/setup/isaac-sim-reference.md` for wiring instructions.

---

## Step 10: Stop When Done

**g6e.2xlarge costs ~$2.40/hour. Stop when not in use.**

```bash
# From your Mac
./scripts/stop-instance.sh

# Or directly:
aws ec2 stop-instances --region us-west-2 \
  --instance-ids $(cd ~/Projects/robotlab/infra/terraform && terraform output -raw instance_id)
```

Resume later with `./scripts/start-instance.sh`.

---

## Deadman Switch (Cost Protection)

The infrastructure includes automatic cost protection to prevent forgotten instances from burning money overnight.

### Three-Layer Defense

1. **Instance-side daemon**: Monitors SSH/DCV connections and GPU usage. If all idle for 30 minutes, shuts down with 10-minute warning.

2. **Nightly Lambda**: Stops the instance at midnight Pacific regardless of activity. You'll receive an email notification.

3. **AWS Budget alerts**: Notifies you at 80% and 100% of monthly EC2 budget ($100 default).

### Enable Notifications

Set your email in `terraform.tfvars` to receive shutdown notifications:

```hcl
deadman_email = "your@email.com"
```

After `terraform apply`, you'll receive a confirmation email from AWS SNS - click to confirm.

### Check Daemon Status (On Instance)

```bash
# Is daemon running?
systemctl status deadman-switch.service

# View recent logs
journalctl -u deadman-switch.service -f

# Manual extend (postpone shutdown)
touch /tmp/deadman-extend
```

### Disable (Not Recommended)

```hcl
deadman_enabled = false
```

---

## Verification Checklist

| Check | Command | Expected |
|-------|---------|----------|
| SSH works | `ssh -i ~/.ssh/robotlab.pem ubuntu@IP` | Shell prompt |
| GPU visible | `nvidia-smi` | L40S, ~46GB |
| DCV connects | `https://IP:8443` | Ubuntu desktop |
| Isaac Sim launches | `/opt/IsaacSim/isaac-sim.sh` | GUI window |
| ROS 2 installed | `ros2 topic list` | `/parameter_events`, `/rosout` |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| SSH/DCV timeout | **IP changed.** Update `my_ip_cidr` in terraform.tfvars, run `terraform apply`. Common on laptops! |
| SSH permission denied | Check key name, run `chmod 400 ~/.ssh/robotlab.pem` |
| DCV login fails | Set password first: `sudo passwd ubuntu` |
| DCV connection refused | Wait 2-3 min after start, check port 8443 open |
| No GPU in nvidia-smi | Wrong instance type - **must use g6e** |
| Isaac Sim "not responding" | Normal during first launch - click "Wait", be patient (5-10 min) |
| ROS topics not visible | Missing OmniGraph wiring, or RMW mismatch |
| RMW mismatch | Set `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in ROS 2 terminal |
| Streaming fails / NVENC error | Wrong GPU type (only L40S has NVENC) |

---

## Cost Summary

| State | Cost |
|-------|------|
| Running (g6e.2xlarge) | ~$2.40/hour |
| Stopped | ~$0.08/GB/month for EBS storage |
| Terminated | Nothing (lose cached shaders) |

**Rule:** A forgotten instance = ~$58/day burned.

---

## Quick Reference

**Before starting:** If you're on a laptop or your IP changes frequently, update `my_ip_cidr` in `terraform.tfvars` before each session:
```bash
# Check/update your IP
MY_IP=$(curl -s -4 ifconfig.me)
echo "my_ip_cidr = \"$MY_IP/32\""
# Edit terraform.tfvars, then: terraform apply -auto-approve
```

| What | Command |
|------|---------|
| Update IP | `cd infra/terraform && terraform apply` (after editing terraform.tfvars) |
| Start instance | `./scripts/start-instance.sh` |
| Stop instance | `./scripts/stop-instance.sh` |
| Get IP | `cd infra/terraform && terraform output public_ip` |
| SSH | `ssh -i ~/.ssh/robotlab.pem ubuntu@IP` |
| DCV | `https://IP:8443` |
| Check GPU | `nvidia-smi` |
| Launch Isaac Sim | See Step 8 above |
| ROS 2 topics | `source /opt/ros/jazzy/setup.bash && ros2 topic list` |

---

## Next Steps

Once the pipeline works:
1. Load a robot scene in Isaac Sim
2. Add OmniGraph nodes for `/clock`, `/cmd_vel`, `/tf`
3. Drive a robot with `ros2 topic pub /cmd_vel ...`
4. See `docs/setup/isaac-sim-reference.md` for detailed OmniGraph instructions
5. See `docs/omnigraph/ros2-patterns.md` for copy-paste patterns
