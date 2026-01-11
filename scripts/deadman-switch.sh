#!/bin/bash
# Deadman Switch for AWS GPU Instance
# Monitors for inactivity and triggers graceful shutdown to prevent runaway costs.
#
# Signals monitored:
#   - SSH sessions (port 22)
#   - DCV sessions (Nice DCV agent)
#   - GPU utilization (nvidia-smi)
#
# Behavior:
#   - Check every POLL_INTERVAL seconds
#   - If all signals idle for IDLE_THRESHOLD seconds, start countdown
#   - If activity resumes during countdown, cancel
#   - If countdown expires, shutdown

set -euo pipefail

# Configuration (can be overridden via environment)
POLL_INTERVAL="${DEADMAN_POLL_INTERVAL:-60}"           # Check every 60 seconds
IDLE_THRESHOLD="${DEADMAN_IDLE_THRESHOLD:-1800}"       # 30 minutes idle before countdown
COUNTDOWN_DURATION="${DEADMAN_COUNTDOWN:-600}"         # 10 minute countdown before shutdown
GPU_IDLE_THRESHOLD="${DEADMAN_GPU_THRESHOLD:-5}"       # GPU utilization below 5% = idle
LOG_FILE="${DEADMAN_LOG_FILE:-/var/log/deadman-switch.log}"
WARNING_FILE="/var/run/deadman-warning"

# State
idle_seconds=0
countdown_seconds=0
in_countdown=false

log() {
    local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $*"
    echo "$msg" | tee -a "$LOG_FILE"
}

# Check if there are active SSH sessions
check_ssh_sessions() {
    # Look for established connections on port 22
    if ss -tnp 2>/dev/null | grep -q ":22.*ESTAB"; then
        return 0  # Active
    fi
    return 1  # Idle
}

# Check if there are active DCV sessions
check_dcv_sessions() {
    # Check if dcv command exists and has sessions
    if command -v dcv &>/dev/null; then
        if dcv list-sessions 2>/dev/null | grep -q "Session"; then
            return 0  # Active
        fi
    fi

    # Fallback: check for DCV agent process with connections
    if pgrep -x "dcvagent" &>/dev/null; then
        return 0  # Active (process running)
    fi

    return 1  # Idle
}

# Check if GPU is being used
check_gpu_utilization() {
    if ! command -v nvidia-smi &>/dev/null; then
        # No nvidia-smi = can't check, assume idle
        return 1
    fi

    local gpu_util
    gpu_util=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')

    if [[ -z "$gpu_util" ]]; then
        return 1  # Can't read, assume idle
    fi

    if (( gpu_util > GPU_IDLE_THRESHOLD )); then
        return 0  # Active
    fi

    return 1  # Idle
}

# Check if system is active (any signal)
is_system_active() {
    if check_ssh_sessions; then
        echo "ssh"
        return 0
    fi
    if check_dcv_sessions; then
        echo "dcv"
        return 0
    fi
    if check_gpu_utilization; then
        echo "gpu"
        return 0
    fi
    return 1
}

# Write warning file (visible in MOTD)
write_warning() {
    local remaining=$1
    local mins=$((remaining / 60))
    local secs=$((remaining % 60))

    cat > "$WARNING_FILE" <<EOF
================================================================================
  DEADMAN SWITCH WARNING

  System has been idle for $((IDLE_THRESHOLD / 60)) minutes.
  Automatic shutdown in: ${mins}m ${secs}s

  To cancel: Resume activity (SSH, DCV, or GPU usage)
  To extend: touch /tmp/deadman-extend
================================================================================
EOF
}

# Clear warning file
clear_warning() {
    rm -f "$WARNING_FILE"
}

# Check for manual extend request
check_extend_request() {
    if [[ -f /tmp/deadman-extend ]]; then
        rm -f /tmp/deadman-extend
        return 0
    fi
    return 1
}

# Main monitoring loop
main() {
    log "Deadman switch starting..."
    log "  Poll interval: ${POLL_INTERVAL}s"
    log "  Idle threshold: ${IDLE_THRESHOLD}s ($((IDLE_THRESHOLD / 60)) min)"
    log "  Countdown duration: ${COUNTDOWN_DURATION}s ($((COUNTDOWN_DURATION / 60)) min)"
    log "  GPU idle threshold: ${GPU_IDLE_THRESHOLD}%"

    while true; do
        # Check for manual extend
        if check_extend_request; then
            log "Manual extend requested, resetting idle timer"
            idle_seconds=0
            in_countdown=false
            countdown_seconds=0
            clear_warning
        fi

        # Check activity
        local activity
        if activity=$(is_system_active); then
            if (( idle_seconds > 0 )) || $in_countdown; then
                log "Activity detected ($activity), resetting idle timer"
            fi
            idle_seconds=0
            in_countdown=false
            countdown_seconds=0
            clear_warning
        else
            idle_seconds=$((idle_seconds + POLL_INTERVAL))

            if (( idle_seconds >= IDLE_THRESHOLD )); then
                if ! $in_countdown; then
                    log "Idle threshold reached (${idle_seconds}s), starting countdown"
                    in_countdown=true
                    countdown_seconds=$COUNTDOWN_DURATION
                fi

                # Update countdown
                countdown_seconds=$((countdown_seconds - POLL_INTERVAL))

                if (( countdown_seconds <= 0 )); then
                    log "Countdown expired, initiating shutdown"
                    clear_warning

                    # Graceful shutdown
                    log "Executing: shutdown -h now"
                    sudo shutdown -h now
                    exit 0
                else
                    write_warning $countdown_seconds
                    log "Shutdown in ${countdown_seconds}s ($(( countdown_seconds / 60 ))m)"
                fi
            fi
        fi

        sleep "$POLL_INTERVAL"
    done
}

# Handle signals
trap 'log "Received SIGTERM, exiting"; clear_warning; exit 0' SIGTERM
trap 'log "Received SIGINT, exiting"; clear_warning; exit 0' SIGINT

main "$@"
