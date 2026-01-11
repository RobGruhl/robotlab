#!/bin/bash
# Bootstrap Claude Code on AWS EC2 instance
# This script is idempotent - safe to run multiple times
#
# Usage: ./bootstrap-claude-code.sh [options]
#   --region REGION       AWS region (default: us-west-2)
#   --repo-url URL        GitHub repo URL
#   --mount PATH          Mount point for persistent volume (default: /mnt/persist)
#   --device PATH         Block device for persistent volume (default: /dev/xvdf)
#   --dry-run             Print what would be done without making changes

set -euo pipefail

# Defaults
REGION="${AWS_REGION:-us-west-2}"
REPO_URL=""
PERSIST_MOUNT="/mnt/persist"
PERSIST_DEVICE="/dev/xvdf"
DRY_RUN=false
SECRET_NAME="robotlab/claude-api-key"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --region) REGION="$2"; shift 2 ;;
    --repo-url) REPO_URL="$2"; shift 2 ;;
    --mount) PERSIST_MOUNT="$2"; shift 2 ;;
    --device) PERSIST_DEVICE="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

run() {
  if $DRY_RUN; then
    log "[DRY-RUN] Would run: $*"
  else
    "$@"
  fi
}

# Step 1: Mount persistent EBS volume
log "Step 1: Checking persistent volume..."
if ! mountpoint -q "$PERSIST_MOUNT"; then
  log "Mounting $PERSIST_DEVICE to $PERSIST_MOUNT"

  # Check if device exists
  if [[ ! -b "$PERSIST_DEVICE" ]]; then
    log "ERROR: Device $PERSIST_DEVICE not found. Is EBS volume attached?"
    exit 1
  fi

  # Format if needed (only if no filesystem exists)
  if ! blkid "$PERSIST_DEVICE" &>/dev/null; then
    log "Formatting $PERSIST_DEVICE as ext4 (first time setup)"
    run sudo mkfs.ext4 -L robotlab-persist "$PERSIST_DEVICE"
  fi

  # Create mount point and mount
  run sudo mkdir -p "$PERSIST_MOUNT"
  run sudo mount "$PERSIST_DEVICE" "$PERSIST_MOUNT"

  # Add to fstab for persistence across reboots
  if ! grep -q "$PERSIST_DEVICE" /etc/fstab; then
    log "Adding to /etc/fstab"
    run bash -c "echo '$PERSIST_DEVICE $PERSIST_MOUNT ext4 defaults,nofail 0 2' | sudo tee -a /etc/fstab"
  fi

  # Set ownership
  run sudo chown -R ubuntu:ubuntu "$PERSIST_MOUNT"
else
  log "Persistent volume already mounted at $PERSIST_MOUNT"
fi

# Step 2: Create directory structure on persistent volume
log "Step 2: Setting up persistent directory structure..."
run mkdir -p "$PERSIST_MOUNT/claude"
run mkdir -p "$PERSIST_MOUNT/robotlab"
run mkdir -p "$PERSIST_MOUNT/rosbags"

# Step 3: Symlink ~/.claude to persistent volume
log "Step 3: Symlinking Claude Code config..."
CLAUDE_DIR="$HOME/.claude"
if [[ -L "$CLAUDE_DIR" ]]; then
  log "Symlink already exists: $CLAUDE_DIR -> $(readlink "$CLAUDE_DIR")"
elif [[ -d "$CLAUDE_DIR" ]]; then
  log "Moving existing .claude to persistent volume"
  run mv "$CLAUDE_DIR" "$PERSIST_MOUNT/claude.backup.$(date +%s)"
  run ln -s "$PERSIST_MOUNT/claude" "$CLAUDE_DIR"
else
  run ln -s "$PERSIST_MOUNT/claude" "$CLAUDE_DIR"
  log "Created symlink: $CLAUDE_DIR -> $PERSIST_MOUNT/claude"
fi

# Step 4: Install Claude Code if not present
log "Step 4: Checking Claude Code installation..."
if command -v claude &>/dev/null; then
  log "Claude Code already installed: $(claude --version 2>/dev/null || echo 'version unknown')"
else
  log "Installing Claude Code..."
  run npm install -g @anthropic-ai/claude-code
fi

# Step 5: Fetch API key from Secrets Manager
log "Step 5: Configuring Claude Code credentials..."
CREDENTIALS_FILE="$PERSIST_MOUNT/claude/credentials.json"
if [[ -f "$CREDENTIALS_FILE" ]]; then
  log "Credentials file already exists"
else
  log "Fetching API key from Secrets Manager..."
  if ! $DRY_RUN; then
    API_KEY=$(aws secretsmanager get-secret-value \
      --region "$REGION" \
      --secret-id "$SECRET_NAME" \
      --query 'SecretString' \
      --output text 2>/dev/null || echo "")

    if [[ -z "$API_KEY" ]]; then
      log "WARNING: Could not fetch API key from Secrets Manager."
      log "         You'll need to run 'claude auth' manually."
      log "         Or set the secret: aws secretsmanager put-secret-value --secret-id $SECRET_NAME --secret-string 'sk-ant-xxx...'"
    else
      # Write credentials in Claude Code format
      echo "{\"apiKey\": \"$API_KEY\"}" > "$CREDENTIALS_FILE"
      chmod 600 "$CREDENTIALS_FILE"
      log "Credentials configured successfully"
    fi
  fi
fi

# Step 6: Clone or update repository
log "Step 6: Setting up repository..."
REPO_DIR="$PERSIST_MOUNT/robotlab"
if [[ -n "$REPO_URL" ]]; then
  if [[ -d "$REPO_DIR/.git" ]]; then
    log "Repository exists, pulling latest..."
    run git -C "$REPO_DIR" pull --ff-only || log "Pull failed, manual intervention may be needed"
  else
    log "Cloning repository..."
    run git clone "$REPO_URL" "$REPO_DIR"
  fi
else
  log "No repo URL specified, skipping clone"
fi

# Create convenience symlink in home directory
if [[ ! -L "$HOME/Projects/robotlab" ]] && [[ -d "$REPO_DIR" ]]; then
  run mkdir -p "$HOME/Projects"
  run ln -sf "$REPO_DIR" "$HOME/Projects/robotlab"
  log "Created symlink: ~/Projects/robotlab -> $REPO_DIR"
fi

# Step 7: Install deadman switch service
log "Step 7: Installing deadman switch service..."
DEADMAN_SCRIPT="$REPO_DIR/scripts/deadman-switch.sh"
DEADMAN_DEST="/opt/robotlab/deadman-switch.sh"
SYSTEMD_SERVICE="/etc/systemd/system/deadman-switch.service"

if [[ -f "$DEADMAN_SCRIPT" ]]; then
  # Copy script to /opt/robotlab
  run sudo mkdir -p /opt/robotlab
  run sudo cp "$DEADMAN_SCRIPT" "$DEADMAN_DEST"
  run sudo chmod +x "$DEADMAN_DEST"

  # Create systemd service file
  if [[ ! -f "$SYSTEMD_SERVICE" ]] || ! $DRY_RUN; then
    log "Creating systemd service for deadman switch"
    run sudo tee "$SYSTEMD_SERVICE" > /dev/null <<'EOF'
[Unit]
Description=Deadman Switch - Auto-shutdown on inactivity
After=network.target

[Service]
Type=simple
ExecStart=/opt/robotlab/deadman-switch.sh
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

# Environment overrides (optional)
# Environment=DEADMAN_IDLE_THRESHOLD=1800
# Environment=DEADMAN_COUNTDOWN=600

[Install]
WantedBy=multi-user.target
EOF
  fi

  # Enable and start service
  run sudo systemctl daemon-reload
  run sudo systemctl enable deadman-switch.service
  run sudo systemctl start deadman-switch.service
  log "Deadman switch service installed and started"
else
  log "WARNING: Deadman switch script not found at $DEADMAN_SCRIPT"
fi

# Step 8: Verify installation
log "Step 8: Verifying installation..."
if command -v claude &>/dev/null; then
  log "Claude Code version: $(claude --version 2>/dev/null || echo 'installed')"
else
  log "WARNING: Claude Code not in PATH"
fi

# Check deadman switch status
if systemctl is-active --quiet deadman-switch.service 2>/dev/null; then
  log "Deadman switch: ACTIVE"
else
  log "Deadman switch: not running (check 'systemctl status deadman-switch.service')"
fi

log ""
log "=== Bootstrap Complete ==="
log ""
log "Next steps:"
log "  1. SSH to instance: ssh ubuntu@<instance-ip>"
log "  2. Start tmux: tmux new -s dev"
log "  3. Navigate to repo: cd ~/Projects/robotlab"
log "  4. Run Claude Code: claude"
log ""
if [[ ! -f "$CREDENTIALS_FILE" ]]; then
  log "  NOTE: Run 'claude auth' to configure API key"
fi
