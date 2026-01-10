#!/bin/bash
# Update Claude Code to the latest version
# Safe to run periodically (e.g., via cron or on instance start)

set -euo pipefail

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

# Check if Claude Code is installed
if ! command -v claude &>/dev/null; then
  log "Claude Code not installed, nothing to update"
  exit 0
fi

# Get current version
CURRENT=$(claude --version 2>/dev/null || echo "unknown")
log "Current version: $CURRENT"

# Check for updates
log "Checking for updates..."
npm update -g @anthropic-ai/claude-code

# Get new version
NEW=$(claude --version 2>/dev/null || echo "unknown")

if [[ "$CURRENT" != "$NEW" ]]; then
  log "Updated: $CURRENT -> $NEW"
else
  log "Already up to date: $NEW"
fi
