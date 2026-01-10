# Claude Code on AWS EC2

This guide covers setting up Claude Code on your AWS Isaac Sim development workstation for optimal robotics development workflow.

## Why Run Claude Code on AWS?

Running Claude Code where your simulator and ROS 2 graph run provides:

- **Tight iteration loop**: "change code → run sim → see result" in one environment
- **No environment mismatch**: Ubuntu + GPU + Isaac Sim is your truth environment
- **Session persistence**: Claude Code history and settings persist across instance stop/start
- **Direct access**: Claude can run ROS commands, inspect logs, and iterate without SSH latency

For detailed rationale, see [Development Workflow](../architecture/development-workflow.md).

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  EC2 Instance (ephemeral root volume)                   │
│  ┌─────────────────────────────────────────────────────┐│
│  │ /home/ubuntu/.claude → symlink to EBS               ││
│  │ /home/ubuntu/Projects/robotlab → EBS mount          ││
│  └─────────────────────────────────────────────────────┘│
│                         │                                │
│  ┌──────────────────────▼──────────────────────────────┐│
│  │  Persistent EBS Volume (/dev/xvdf → /mnt/persist)   ││
│  │  ├── claude/          # CC config + session history ││
│  │  ├── robotlab/        # Git repo clone              ││
│  │  └── rosbags/         # Recorded data               ││
│  └─────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────┘
                          │
     ┌────────────────────▼────────────────────┐
     │  AWS Secrets Manager                    │
     │  └── robotlab/claude-api-key            │
     └─────────────────────────────────────────┘
```

Key components:
- **Persistent EBS**: Survives instance stop/start/termination
- **Secrets Manager**: Securely stores API key, fetched on boot
- **User-data bootstrap**: Automatic setup on first boot

## First-Time Setup

### 1. Store Your API Key in Secrets Manager

Before deploying infrastructure:

```bash
# Get your API key from https://console.anthropic.com/
aws secretsmanager put-secret-value \
  --region us-west-2 \
  --secret-id robotlab/claude-api-key \
  --secret-string "sk-ant-api03-YOUR-KEY-HERE"
```

### 2. Deploy Infrastructure

```bash
cd infra/terraform

# Initialize terraform
terraform init

# Create terraform.tfvars with your values
cat > terraform.tfvars <<EOF
my_ip_cidr      = "$(curl -s ifconfig.me)/32"
key_name        = "your-aws-keypair"
isaac_sim_ami   = "ami-xxxxxxxxx"  # From AWS Marketplace
github_repo_url = "https://github.com/YOUR_USERNAME/robotlab.git"
EOF

# Deploy
terraform apply
```

### 3. Connect and Verify

```bash
# SSH to instance
ssh -i ~/.ssh/your-key.pem ubuntu@$(terraform output -raw public_ip)

# Verify Claude Code is installed
claude --version

# Navigate to repo
cd ~/Projects/robotlab

# Start Claude Code
claude
```

## Daily Workflow

### Starting Your Day

```bash
# Start the instance (if stopped)
./scripts/start-instance.sh

# SSH and attach to tmux
ssh ubuntu@<ip>
tmux attach -t dev || tmux new -s dev

# Start Claude Code in repo
cd ~/Projects/robotlab
claude
```

### Ending Your Day

```bash
# Detach from tmux (Ctrl-b d)
# Stop instance to save money
./scripts/stop-instance.sh
```

### Resuming a Session

Claude Code maintains session history on the persistent volume:

```bash
# Continue last conversation
claude --continue

# Resume a specific session
claude --resume
```

## Manual Bootstrap (If Needed)

If user-data doesn't run or you need to re-run:

```bash
# Copy bootstrap script to instance
scp scripts/bootstrap-claude-code.sh ubuntu@<ip>:/opt/robotlab/

# SSH and run
ssh ubuntu@<ip>
sudo /opt/robotlab/bootstrap-claude-code.sh \
  --region us-west-2 \
  --repo-url https://github.com/YOUR_USERNAME/robotlab.git
```

## Updating Claude Code

Updates are automatic via the bootstrap script, but you can manually update:

```bash
./scripts/update-claude-code.sh
```

Or add to crontab for daily updates:

```bash
# Update at 9am daily
0 9 * * * /home/ubuntu/Projects/robotlab/scripts/update-claude-code.sh >> /var/log/claude-update.log 2>&1
```

## Troubleshooting

### Claude Code Not Installed

```bash
# Check npm installation
which npm
npm --version

# Install Claude Code manually
npm install -g @anthropic-ai/claude-code
```

### API Key Not Working

```bash
# Verify secret exists
aws secretsmanager get-secret-value \
  --region us-west-2 \
  --secret-id robotlab/claude-api-key

# Re-run bootstrap to fetch credentials
sudo /opt/robotlab/bootstrap-claude-code.sh --region us-west-2

# Or authenticate manually
claude auth
```

### Persistent Volume Not Mounted

```bash
# Check if volume is attached
lsblk

# Mount manually
sudo mount /dev/xvdf /mnt/persist

# Verify
df -h /mnt/persist
```

### Session History Lost

Session history is stored in `~/.claude` which symlinks to `/mnt/persist/claude`. Verify:

```bash
ls -la ~/.claude
ls -la /mnt/persist/claude
```

## Security Notes

- API key is stored in AWS Secrets Manager, never in code or AMI
- Instance role has minimal permissions (only `secretsmanager:GetSecretValue` for `robotlab/*`)
- SSH access restricted to your IP via security group
- Credentials file on disk is mode 600 (owner-only read/write)

## Cost Management

The persistent EBS volume incurs cost even when instance is stopped (~$0.08/GB/month for gp3):

- 100GB volume = ~$8/month

To destroy everything including persistent data:

```bash
# WARNING: This deletes all data including Claude sessions and rosbags
cd infra/terraform
terraform destroy
```

To keep data but reduce costs, just stop the instance:

```bash
./scripts/stop-instance.sh
```
