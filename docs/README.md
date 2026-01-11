# Robotlab Documentation

## Development Model

**Laptop** = thin client running Claude Code for coordination and research
**AWS** = GPU instance (g6e) running Isaac Sim + Claude Code for simulation work

Both instances share state via `.claude/claude-sync.md` and the git repo.

---

## I want to...

### Get started from scratch

1. [AWS Deployment Guide](getting-started/aws-deployment.md) - Launch Isaac Sim on AWS, connect via DCV
2. [Isaac Sim Reference](setup/isaac-sim-reference.md) - Detailed setup, OmniGraph wiring, troubleshooting

### Work from my laptop (thin client)

1. Read [CLAUDE.md](../CLAUDE.md) - Project context and conventions
2. Check [.claude/claude-sync.md](../.claude/claude-sync.md) - Current AWS state
3. Push research/instructions for AWS Claude to pick up

### Work on AWS instance (simulation)

1. Pull latest from git (get laptop updates)
2. Reference [OmniGraph Patterns](omnigraph/ros2-patterns.md) - Robot wiring
3. Update [.claude/claude-sync.md](../.claude/claude-sync.md) - Share findings, push

### Understand the architecture

- [Architecture Overview](architecture/README.md) - Layers and principles
- [Topic Conventions](architecture/topic-conventions.md) - ROS 2 contracts
- [Drone Patterns](architecture/drone-patterns.md) - Offboard control, safety
- [LLM Integration](architecture/llm-integration.md) - Prompts, schemas, validation

### Check project status

- [Roadmap](roadmap/README.md) - Vision, demo ladder, phases
- [Backlog](roadmap/backlog.md) - Current work, hardware research

---

## Directory Index

| Directory | Purpose |
|-----------|---------|
| [getting-started/](getting-started/) | Onboarding tutorials |
| [setup/](setup/) | Detailed setup guides |
| [architecture/](architecture/) | System design, patterns |
| [omnigraph/](omnigraph/) | Isaac Sim visual scripting reference |
| [roadmap/](roadmap/) | Planning, backlog |
