#!/bin/bash
# User-data script for robotlab Isaac Sim instance
# Runs on first boot and after any user-data changes

set -euo pipefail
exec > >(tee /var/log/user-data.log) 2>&1

echo "=== RobotLab Bootstrap Started: $(date) ==="

# Variables from terraform template
REGION="${region}"
GITHUB_REPO_URL="${github_repo_url}"
PERSIST_MOUNT="/mnt/persist"
PERSIST_DEVICE="/dev/xvdf"

# Run bootstrap as ubuntu user
sudo -u ubuntu /opt/robotlab/bootstrap-claude-code.sh \
  --region "$REGION" \
  --repo-url "$GITHUB_REPO_URL" \
  --mount "$PERSIST_MOUNT" \
  --device "$PERSIST_DEVICE"

echo "=== RobotLab Bootstrap Complete: $(date) ==="
