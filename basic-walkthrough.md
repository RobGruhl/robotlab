# Basic Walkthrough: Your First Isaac Sim Session on AWS

This guide takes you from zero to seeing a 3D robot simulator running on a powerful cloud GPU, streaming to your Mac. Every step includes validation so you know it worked.

---

## What You'll Have When Done

By the end of this walkthrough, you'll have:

1. **A terminal window** running Claude Code locally for planning and local commands
2. **NICE DCV client** showing a full Ubuntu desktop with Isaac Sim running
3. **ROS 2 topics** flowing that you can inspect and eventually control

Here's the mental model:

```
YOUR MAC                                    AWS CLOUD
┌─────────────────────┐                    ┌─────────────────────┐
│                     │                    │                     │
│  Terminal 1         │                    │                     │
│  Claude Code        │  (stays local -    │                     │
│  (this terminal!)   │   for now)         │                     │
│                     │                    │                     │
├─────────────────────┤                    ├─────────────────────┤
│                     │                    │  ┌───────────────┐  │
│                     │    NICE DCV        │  │ Ubuntu Desktop│  │
│  DCV Client         │◀══(streaming)═════ │  │ ┌───────────┐ │  │
│  (full desktop!)    │   (port 8443)      │  │ │Isaac Sim  │ │  │
│                     │                    │  │ │  window   │ │  │
│                     │                    │  │ └───────────┘ │  │
│                     │                    │  │ + Terminal    │  │
│                     │                    │  └───────────────┘  │
│                     │                    │                     │
└─────────────────────┘                    └─────────────────────┘
```

**Your window layout on Mac:**
- **Terminal 1:** Claude Code (this conversation) - for planning, questions, and local commands
- **DCV Client:** Full Ubuntu desktop with Isaac Sim GUI + terminals for ROS 2 commands

**Key insight:** You see a **full remote desktop** via NICE DCV, not just a video stream. You can:
- Open terminals inside the remote desktop
- Run Isaac Sim with full GUI
- Use all Ubuntu apps

**Later:** Once everything works, you can run Claude Code *on* the AWS instance for tighter iteration. But for this walkthrough, keep Claude Code local so I can help you troubleshoot.

---

## Prerequisites Checklist

Before starting, you need these installed on your Mac. Let's verify each one.

### Check 1: AWS CLI

Open Terminal (Cmd+Space, type "Terminal", press Enter).

```bash
aws --version
```

**You should see:** Something like `aws-cli/2.x.x Python/3.x.x Darwin/...`

**If you don't see this:**
```bash
brew install awscli
```

### Check 2: AWS CLI is configured

```bash
aws sts get-caller-identity
```

**You should see:** JSON with your `Account` number and `Arn`

**If you get an error:**
```bash
aws configure
# Enter your Access Key ID, Secret Access Key, region (us-west-2), and output format (json)
```

### Check 3: Terraform

```bash
terraform --version
```

**You should see:** `Terraform v1.x.x`

**If you don't see this:**
```bash
brew install terraform
```

### Check 4: You have an SSH key pair in AWS

```bash
aws ec2 describe-key-pairs --region us-west-2
```

**You should see:** JSON with at least one `KeyName`

**If KeyPairs is empty `[]`:** You need to create one. Do this:

```bash
# Create a new key pair and save it locally
aws ec2 create-key-pair \
  --key-name robotlab-key \
  --region us-west-2 \
  --query 'KeyMaterial' \
  --output text > ~/.ssh/robotlab-key.pem

# Set correct permissions (required for SSH to work)
chmod 400 ~/.ssh/robotlab-key.pem

# Verify it was created
aws ec2 describe-key-pairs --region us-west-2
```

**You should now see:** `"KeyName": "robotlab-key"` in the output

**Write down your key name:** _____________ (you'll need this later)

### Check 5: Claude API Key is in Secrets Manager

```bash
aws secretsmanager describe-secret --region us-west-2 --secret-id robotlab/claude-api-key
```

**You should see:** JSON with `"Name": "robotlab/claude-api-key"`

**If you get "ResourceNotFoundException":** Go back and create the secret (see claude-code-aws.md)

---

## Step 1: Find Your Isaac Sim AMI ID

Since you're already subscribed to Isaac Sim in AWS Marketplace, you just need to find the AMI ID.

```bash
aws ec2 describe-images \
  --region us-west-2 \
  --owners aws-marketplace \
  --filters "Name=name,Values=*Isaac*Sim*" \
  --query 'Images[*].[ImageId,Name,CreationDate]' \
  --output table
```

**You should see:** A table with AMI IDs like `ami-0abc123...`

**Write down the most recent AMI ID:** _________________________

**If you see nothing:** Your marketplace subscription might be in a different region, or not fully activated. Go to AWS Marketplace → Your Marketplace Software → Find Isaac Sim → Verify subscription is active.

---

## Step 2: Navigate to the Terraform Directory

```bash
cd ~/Projects/robotlab/infra/terraform
```

**You should see:** No error, and if you run `ls`, you see `main.tf`, `variables.tf`, etc.

**If directory doesn't exist:** Clone the repo first:
```bash
git clone <your-repo-url> ~/Projects/robotlab
cd ~/Projects/robotlab/infra/terraform
```

---

## Step 3: Create Your Configuration File

Create a file called `terraform.tfvars` with your specific values:

```bash
cat > terraform.tfvars << 'EOF'
my_ip_cidr      = "YOUR_IP/32"
key_name        = "YOUR_KEY_NAME"
isaac_sim_ami   = "YOUR_AMI_ID"
region          = "us-west-2"
github_repo_url = "https://github.com/YOUR_USERNAME/robotlab.git"

# IMPORTANT: Isaac Sim AMI requires g6e instances (L40S GPU)
use_spot      = false
instance_type = "g6e.xlarge"
EOF
```

Now replace the placeholders:

```bash
# Get your current IP (use -4 to force IPv4)
MY_IP=$(curl -s -4 ifconfig.me)
sed -i '' "s|YOUR_IP|$MY_IP|g" terraform.tfvars

# Verify the file
cat terraform.tfvars
```

**You should see:** The file with your actual IP address filled in.

**Now manually edit to fill in the other values:**

```bash
nano terraform.tfvars
# Or use: open -e terraform.tfvars
```

Replace:
- `YOUR_KEY_NAME` → the SSH key name you wrote down (e.g., `robotlab`)
- `YOUR_AMI_ID` → the AMI ID you wrote down (e.g., `ami-0a58578db493e70fb`)
- `YOUR_USERNAME` → your GitHub username

Save and exit (Ctrl+X, then Y, then Enter in nano).

**Verify your changes:**
```bash
cat terraform.tfvars
```

**You should see:** All values filled in, no "YOUR_" placeholders remaining.

**Critical settings:**
- `instance_type = "g6e.xlarge"` - **Required**. Other GPU types won't work.
- `use_spot = false` - **Required**. Spot instances not supported for this AMI.

---

## Step 4: Initialize Terraform

```bash
terraform init
```

**You should see:**
```
Terraform has been successfully initialized!
```

**If you see errors about providers:** Run `terraform init -upgrade`

---

## Step 5: Preview What Will Be Created

```bash
terraform plan
```

**You should see:** A list of resources to be created, ending with something like:
```
Plan: 7 to add, 0 to change, 0 to destroy.
```

**If you see errors:** Read them carefully - usually it's a typo in terraform.tfvars.

---

## Step 6: Deploy the Infrastructure

This is the big moment. This will create real AWS resources that cost money.

```bash
terraform apply
```

**You should see:** The same plan, followed by:
```
Do you want to perform these actions?
```

**Type:** `yes` (and press Enter)

**Wait time:** About 2-3 minutes

**You should see:**
```
Apply complete! Resources: 7 added, 0 changed, 0 destroyed.

Outputs:

public_ip = "XX.XX.XX.XX"
```

**Write down the public IP:** _________________________

**If it fails:** Read the error. Common issues:
- "UnsupportedOperation" → **Wrong instance type**. Isaac Sim AMI requires g6e instances only.
- "UnauthorizedOperation" → Your AWS account can't create this instance type
- "InsufficientInstanceCapacity" → Try a different availability zone
- "VcpuLimitExceeded" → You need to request a quota increase for GPU instances

---

## Step 7: Wait for Instance to Initialize

The instance needs a minute to boot up. Let's check its status:

```bash
aws ec2 describe-instance-status \
  --region us-west-2 \
  --instance-ids $(terraform output -raw instance_id) \
  --query 'InstanceStatuses[0].InstanceState.Name' \
  --output text
```

**Keep running this until you see:** `running`

**If it stays "pending" for more than 5 minutes:** Something's wrong. Check the AWS Console.

---

## Step 8: SSH to Your Instance

Now let's connect! Replace `XX.XX.XX.XX` with your actual IP and `robotlab-key` with your key name:

```bash
ssh -i ~/.ssh/robotlab-key.pem ubuntu@XX.XX.XX.XX
```

**First time connecting?** You'll see:
```
The authenticity of host 'XX.XX.XX.XX' can't be established.
Are you sure you want to continue connecting (yes/no)?
```

**Type:** `yes`

**You should see:** A welcome message and a prompt like `ubuntu@ip-10-0-1-123:~$`

**If "Permission denied":**
- Check your key name matches what's in terraform.tfvars
- Check the key file permissions: `chmod 400 ~/.ssh/robotlab-key.pem`

**If "Connection refused":**
- The instance might still be booting. Wait 1 minute and try again.

**If "Operation timed out":**
- Your IP might have changed. Get it with `curl -s ifconfig.me`
- Update terraform.tfvars and run `terraform apply` again

---

## Step 9: Verify the GPU Works

Now you're on the remote machine! Let's check the GPU:

```bash
nvidia-smi
```

**You should see:** A table showing the GPU, something like:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.xx       Driver Version: 535.xx       CUDA Version: 12.x    |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA L40S         On   | 00000000:00:1E.0 Off |                    0 |
| N/A   30C    P8    22W / 350W |      0MiB / 46068MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
```

**If "command not found" or no GPU shown:** You're on the wrong instance type. Terminate and recreate with a GPU instance.

---

## Step 10: Download NICE DCV Client (On Your Mac)

NICE DCV is Amazon's remote desktop protocol - it streams the full Ubuntu desktop to your Mac.

1. Go to: https://download.nice-dcv.com/
2. Download the **macOS client** (`.dmg` file)
3. Install it (drag to Applications)

**You should see:** NICE DCV Viewer in your Applications folder

**Approximate time:** 2-3 minutes

---

## Step 11: Set Ubuntu Password (Required for DCV)

Before you can log into DCV, you must set a password for the ubuntu user. SSH to your instance:

```bash
ssh -i ~/.ssh/robotlab.pem ubuntu@XX.XX.XX.XX
```

Then set the password:

```bash
sudo passwd ubuntu
```

Enter a password when prompted (you'll type it twice). **Remember this password** - you'll need it for DCV login.

---

## Step 12: Connect with DCV (The Payoff!)

On your **Mac**, open the NICE DCV Viewer:

1. Open NICE DCV Viewer from Applications
2. Enter the connection URL: `https://XX.XX.XX.XX:8443` (your AWS IP)
3. Click **Connect**
4. You'll see a certificate warning - click **Trust** or **Connect Anyway**
5. Log in with:
   - Username: `ubuntu`
   - Password: the password you just set

**You should see:** The full Ubuntu desktop with NVIDIA green wallpaper!

**If "Connection refused":**
- Make sure port 8443 is open in your security group (Terraform does this)
- The instance might still be booting - wait 1 minute

**If login fails:**
- Make sure you set the password via SSH first (`sudo passwd ubuntu`)

---

## Step 13: Launch Isaac Sim (On the Remote Desktop)

Now you're looking at the Ubuntu desktop via DCV. Let's launch Isaac Sim:

1. **Open Terminal** - click the app grid (bottom-left of dock) and find "Terminal"
2. **Run Isaac Sim:**
   ```bash
   /opt/IsaacSim/isaac-sim.sh
   ```

**You should see:** Lots of log output in the terminal, then the Isaac Sim GUI window appears.

**First launch takes 2-5 minutes** while it compiles shaders. You may see "Isaac Sim is not responding" dialogs - **just click "Wait"** each time. This is normal during initialization.

**When it's ready:** You'll see the Isaac Sim interface with:
- A dark 3D viewport
- Content browser showing Robots, Environments, IsaacLab folders
- Menu bar with File, Edit, Create, etc.

**Good news:** Shaders are cached to disk, so subsequent launches are much faster (30-60 seconds).

---

## Step 14: Verify ROS 2 Topics (Optional But Recommended)

Open another **Terminal** inside the DCV remote desktop (or SSH from your Mac):

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash   # or jazzy, depending on AMI

# List available topics
ros2 topic list
```

**You should see:** Topics like:
```
/clock
/parameter_events
/rosout
```

**If you see camera or robot topics too**, that means a simulation scene with ROS bridge is running - even better!

**If "command not found":** ROS 2 might not be installed on this AMI, or you need to source a different setup file.

---

## Step 15: Celebrate, Then STOP THE INSTANCE

You did it! You're running a 3D robot simulator on a cloud GPU!

**But this costs ~$1.20/hour (g6e.xlarge).** Let's stop it:

1. **In Isaac Sim:** File → Exit (or close the window)
2. **Close the DCV client** on your Mac
3. **On your Mac (local terminal):** Run:

```bash
aws ec2 stop-instances --region us-west-2 --instance-ids $(cd ~/Projects/robotlab/infra/terraform && terraform output -raw instance_id)
```

Or if you know your instance ID:
```bash
aws ec2 stop-instances --region us-west-2 --instance-ids i-0123456789abcdef0
```

**Verify it stopped:**
```bash
aws ec2 describe-instances \
  --region us-west-2 \
  --query 'Reservations[*].Instances[*].[InstanceId,State.Name]' \
  --output table
```

**You should see:** `stopping` then `stopped`

---

## What's Next?

Now that you've validated the pipeline works, here's what's ahead:

1. **Start the instance again** when you want to work:
   ```bash
   ./scripts/start-instance.sh
   ```

2. **Load a robot scene** in Isaac Sim (via the streaming client)

3. **Enable ROS 2 bridge** in that scene

4. **Run Claude Code on the AWS instance** for the best development experience

5. **Record rosbags** for debugging and replay

---

## Quick Reference

| What | Command |
|------|---------|
| Start instance | `aws ec2 start-instances --region us-west-2 --instance-ids INSTANCE_ID` |
| Stop instance | `aws ec2 stop-instances --region us-west-2 --instance-ids INSTANCE_ID` |
| Get instance IP | `cd ~/Projects/robotlab/infra/terraform && terraform output public_ip` |
| SSH in | `ssh -i ~/.ssh/robotlab.pem ubuntu@IP` |
| Connect DCV | Open NICE DCV Viewer → `https://IP:8443` → login as `ubuntu` |
| Check GPU | `nvidia-smi` (on AWS) |
| Start Isaac Sim | `/opt/IsaacSim/isaac-sim.sh` |
| List ROS topics | `ros2 topic list` (after `source /opt/ros/humble/setup.bash`) |

---

## Troubleshooting Summary

| Problem | Solution |
|---------|----------|
| Can't SSH - timeout | Your IP changed. Update terraform.tfvars `my_ip_cidr`, run `terraform apply` |
| Can't SSH - permission denied | Check key name and permissions (`chmod 400`) |
| DCV login fails | Must set password first: SSH in, run `sudo passwd ubuntu` |
| DCV won't connect | Check port 8443 is open, instance is running |
| No GPU in nvidia-smi | Wrong instance type - **must use g6e.xlarge or g6e.2xlarge** |
| Isaac Sim "not responding" | **Normal during first launch** - click "Wait", be patient (2-5 min) |
| Isaac Sim won't start | Check path: `/opt/IsaacSim/isaac-sim.sh` |
| ROS topics empty | Load a scene with ROS bridge enabled |
| Costs too much | **Stop the instance when not using it!** |

---

## Important: Instance Type Requirements

The Isaac Sim AMI **only works with g6e instances** (L40S GPU). Other GPU types will fail:

| Instance Type | Works? | Notes |
|---------------|--------|-------|
| g6e.xlarge | **Yes** | Recommended for dev (~$1.20/hr) |
| g6e.2xlarge | **Yes** | More CPU/RAM (~$1.50/hr) |
| g5.xlarge | No | AMI rejects it |
| g4dn.xlarge | No | AMI rejects it |

**Spot instances are NOT supported** for this AMI.

---

## Cost Summary

| State | Cost |
|-------|------|
| Running (g6e.xlarge) | ~$1.20/hour |
| Running (g6e.2xlarge) | ~$1.50/hour |
| Stopped | ~$0.08/GB/month for storage only |
| Terminated | Nothing (but you lose cached shaders) |

**Rule of thumb:** Always stop when you're done for the day. A forgotten instance = $29/day burned.
